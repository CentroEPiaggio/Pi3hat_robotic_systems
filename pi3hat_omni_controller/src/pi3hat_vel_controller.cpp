#include "pluginlib/class_list_macros.hpp"
#include "pi3hat_omni_controller/pi3hat_vel_controller.hpp"
#include "pi3hat_hw_interface/motor_manager.hpp"
#include <cstdint>
#include "eigen3/Eigen/Core"
#define VEL_CMD true
#define MAX_LOSS 20
#define DEBUG true
#include <cmath>
namespace pi3hat_vel_controller
{
    
    Pi3Hat_Vel_Controller::Pi3Hat_Vel_Controller():
    logger_name_("Pi3Hat_Vel_Controller"),
    //rt_buffer_(nullptr),
    vel_target_rcvd_msg_(make_shared<CmdMsgs>())
    //prova
    {}


    CallbackReturn Pi3Hat_Vel_Controller::on_init()
    {


        // declare homing duration params
        try
        {
            auto_declare<double>("homing_duration",5.0); 
            auto_declare<int>("input_frequency",100);   
            auto_declare<double>("driveshaft_y",0.188);
            auto_declare<double>("driveshaft_x",0.235);
            auto_declare<double>("mecanum_angle",45.0);
	        auto_declare<double>("wheel_rad",0.04);
            auto_declare<double>("init_height",-0.22);
            auto_declare<double>("max_heigth",-0.37);
            auto_declare<double>("min_height",-0-15);
        }
         catch(const std::exception & e)
        {
            RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"Exception thrown during declaration of joints name with message: %s", e.what());
            return CallbackReturn::ERROR;
        }
        // add physics omni_wheel physics parameter autodeclare
        RCLCPP_INFO(get_node()->get_logger(),"initialize succesfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Pi3Hat_Vel_Controller::on_configure(const rclcpp_lifecycle::State &)
    {
        std::vector<double> init_positions;
        rclcpp::QoS sub_qos(10);
        rclcpp::SubscriptionOptions sub_opt;
        double homig_dur, rf_hfe_hom,rf_kfe_hom;
        size_t sz;
        // get the controlled joints name
        a_ = get_node()->get_parameter("driveshaft_y").as_double();
        b_ = get_node()->get_parameter("driveshaft_x").as_double();
        alpha_ = get_node()->get_parameter("mecanum_angle").as_double() *(M_PI/180.0);
	    r_ = get_node()->get_parameter("wheel_rad").as_double();
        init_height_ = get_node()->get_parameter("init_height").as_double();
        max_height_ = get_node()->get_parameter("max_heigth").as_double();
        min_height_ = get_node()->get_parameter("min_height").as_double();
        std::chrono::duration dur = std::chrono::milliseconds(get_node()->get_parameter("input_frequency").as_int());
        act_height_ = init_height_;
        //get homing duration 

        homing_dur_ = get_node()->get_parameter("homing_duration").as_double();
        RCLCPP_INFO(get_node()->get_logger(),"homing dur is  %f",homing_dur_);


        IK_RF(rf_hfe_hom,rf_kfe_hom,init_height_);



        // set spline parameters
        spline_par_[0] = (3.0 * rf_hfe_hom) / (homing_dur_*homing_dur_); // a_2_hip
        spline_par_[1] = (-2.0 * rf_hfe_hom )/( homing_dur_*homing_dur_*homing_dur_); // a_3_hip
        spline_par_[2] = (3.0 * rf_kfe_hom )/ (homing_dur_*homing_dur_); // a_2_knee
        spline_par_[3] = (-2.0 * rf_kfe_hom) /( homing_dur_*homing_dur_*homing_dur_); // a_3_knee

        RCLCPP_INFO(get_node()->get_logger(),"the spline vars are %f,%f and %f",spline_par_[1],spline_par_[0],(3.0 * RF_HFE_HOM) / (homing_dur_*homing_dur_));
        
        // fill the map structure 
        sz = joints_.size();
        for(size_t i = 0; i < sz; i++)
        {
            // da mettere joints_ al posto di joint nuovo membro 
            // position_cmd_.emplace(std::make_pair(joints_[i],i<(JNT_LEG_NUM)*LEG_NUM?0.0:std::nan("0"))); // init with NaN
            position_cmd_.emplace(std::make_pair(joints_[i],0.0)); // init with NaN
            position_out_.emplace(std::make_pair(joints_[i],std::nan("0")));                //used to store current measured joint position
            temperature_out_.emplace(std::make_pair(joints_[i],std::nan("0")));             //used to store current measured joint temperature
            velocity_cmd_.emplace(std::make_pair(joints_[i],0.0));
            effort_cmd_.emplace(std::make_pair(joints_[i],0.0));
            kp_scale_cmd_.emplace(std::make_pair(joints_[i],1.0)); 
            kd_scale_cmd_.emplace(std::make_pair(joints_[i],1.0));

        }
        sub_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        sub_qos.deadline();
        sub_opt.event_callbacks.deadline_callback = [&](rclcpp::QOSDeadlineRequestedInfo& event)->void
        {
            RCLCPP_INFO(get_node()->get_logger(),"miss subscriber deadline: %d", event.total_count);
            loss_counter_ ++;
            if(loss_counter_ > MAX_LOSS)
            {
                for(auto &it: velocity_cmd_)
                {
                    it.second = 0.0;
                }
            }
        };
        state_ = Controller_State::PRE_HOMING;
        cmd_sub_ = get_node()->create_subscription<CmdMsgs>(
            "~/command",
            sub_qos,
            [this](const CmdMsgs::SharedPtr msg)
            {
                if(state_ == Controller_State::ACTIVE)
                {
                // aggiungere il lock_guard std::lock_guard(<mutex_var_>)
                    lock_guard<std::mutex> lock(mutex_var_);
                    loss_counter_ = 0;
                    // RCLCPP_INFO(get_node()->get_logger(),"PPP");
                    vel_target_rcvd_msg_->set__v_x(msg->v_x);
                    vel_target_rcvd_msg_->set__v_y(msg->v_y);
                    vel_target_rcvd_msg_->set__omega(msg->omega);
                    vel_target_rcvd_msg_->set__height_rate(0.0);
                    // vel_target_rcvd_msg_->set__height_rate(msg->height_rate);
                }
            }
        );
        #if DEBUG
            pub_deb_ = get_node()->create_publisher<DebugMsgs>("/joint_states",10);
            wheel_pub_ = get_node()->create_publisher<DebugMsgs>("/wheel_states",10);
        #endif
        rclcpp::ServicesQoS srvs_qos;

        // srvs_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);

        homing_serv_ = get_node()->create_service<TransactionService>("~/homing_srv",
        std::bind(&Pi3Hat_Vel_Controller::homing_start_srv,this,std::placeholders::_1,std::placeholders::_2),
        srvs_qos.get_rmw_qos_profile()
        );

        emergency_serv_ = get_node()->create_service<TransactionService>("~/emergency_erv",
        std::bind(&Pi3Hat_Vel_Controller::emergency_srv,this,std::placeholders::_1,std::placeholders::_2),
        srvs_qos.get_rmw_qos_profile()
        );

        RCLCPP_INFO(get_node()->get_logger(),"configure succesfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Pi3Hat_Vel_Controller::on_activate(const rclcpp_lifecycle::State &)
    {
       // rt_buffer_.reset();
       // JACOPINO se c'è tempo scrivi un check sulle interfacce hostate e claimate 
        RCLCPP_INFO(get_node()->get_logger(),"activate succesfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Pi3Hat_Vel_Controller::on_deactivate(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Pi3Hat_Vel_Controller::on_cleanup(const rclcpp_lifecycle::State &)
    {
        return CallbackReturn::SUCCESS;
    }
    
    bool Pi3Hat_Vel_Controller::get_target(double& v_x_tmp, double& v_y_tmp, double& omega_tmp, double& height_rate_tmp)
    { 
        // joints_rcvd_msg_ = *rt_buffer_.readFromRT();

        //  lock_guard + save data on var
        std::lock_guard<std::mutex> lock(mutex_var_);        //when exit from get_target() the mutex is unlocked

        
        try
        {   
            // RCLCPP_INFO(get_node()->get_logger(),"pass name %s",joints_rcvd_msg_->name[i].c_str());
            v_x_tmp = vel_target_rcvd_msg_->v_x;
            v_y_tmp = vel_target_rcvd_msg_->v_y;
            omega_tmp = vel_target_rcvd_msg_->omega;
            height_rate_tmp = vel_target_rcvd_msg_->height_rate;
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR( rclcpp::get_logger(logger_name_),"Raised error during the velocity targets assegnation %s", e.what());
            return false;
        }
        return true;
            
    }

    void Pi3Hat_Vel_Controller::update_base_height(double dh, double dur)
    {
        act_height_ += dur*dh;
        if(act_height_ > max_height_)
            act_height_ = max_height_ ;
        if(act_height_ < min_height_)
            act_height_ = min_height_;
    }

    bool Pi3Hat_Vel_Controller::compute_reference(double v_x_tmp, double v_y_tmp, double omega_tmp, double height_rate_tmp, double dt)// add duration as argument [s]
    {   
        VectorXd v_base(3);
        VectorXd w_mecanum(4);
        v_base << v_x_tmp, v_y_tmp, omega_tmp;


        update_base_height(height_rate_tmp,dt);



        compute_mecanum_speed(v_base, w_mecanum);
        // RCLCPP_INFO(get_node()->get_logger(),"the wheel is [%f,%f,%f,%f]",w_mecanum[0],w_mecanum[1],w_mecanum[2],w_mecanum[3]);
        //update wheels_velocity_cmd map, last four elements of the joint list
        for (size_t i = LEG_NUM * JNT_LEG_NUM; i < LEG_NUM * JNT_LEG_NUM + WHL_NUM; i++)
        {
            try
            {   
                velocity_cmd_.at(joints_[i]) = w_mecanum[i - JNT_LEG_NUM * LEG_NUM];

                // RCLCPP_INFO(get_node()->get_logger(),"the %ld jnt is %s and the vel is %f",i,joints_[i].c_str(),w_mecanum[i - JNT_LEG_NUM * LEG_NUM]);
                #if VEL_CCM 
                    position_cmd_.at(joints_[i]) = std::nan();
                #else
                    position_cmd_.at(joints_[i]) += dt* velocity_cmd_.at(joints_[i]);
                #endif 
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR( rclcpp::get_logger(logger_name_),"Raised error during the wheel velocity references assegnation %s", e.what());
                return false;
            }            
        }
        
        for (auto &i:legs_)
        {   
            VectorXd q_leg(2), q_dot_leg(2);   //[ HFE, KFE]
            
            //extract the i-th leg joint position info
            for (size_t j = 0; j < JNT_LEG_NUM; j++)
            {
                RCLCPP_INFO(get_node()->get_logger(),"joint name is %s and its index is %ld",joints_[JNT_LEG_NUM*i + j].c_str(),JNT_LEG_NUM*i + j);
                q_leg(j) = position_out_.at(joints_[JNT_LEG_NUM*i + j]) ; 
                // q_leg_cmd(j) = position_cmd_.at(joints_[JNT_LEG_NUM*i + j]);

            }
            
           

            // coumpute acc
            
            compute_leg_joints_vel_ref(q_leg, q_dot_leg, i, height_rate_tmp);

             // add compute IK separated per LEG
            if(i == LEG_IND::RF || i == LEG_IND::LH)
            {

            }
            
            //insert the i-th leg joint velocity reference
            for (size_t j = 0; j < JNT_LEG_NUM; j++)
            {
                try
                {   
                    // add integration of pos with computed command vel, so we can send also position reference  
                    position_cmd_.at(joints_[JNT_LEG_NUM*i + j]) += q_dot_leg(j) * dt ;
                    velocity_cmd_.at(joints_[JNT_LEG_NUM*i + j]) = q_dot_leg(j); 
                }
                catch(const std::exception& e)
                {
                    RCLCPP_ERROR( rclcpp::get_logger(logger_name_),"Raised error during the %d-th leg %ld-th joint references assegnation %s", i, j, e.what());
                    return false;
                }              
            }
        }
        

        return true;
    }

    void Pi3Hat_Vel_Controller::compute_mecanum_speed(VectorXd& v_base, VectorXd& w_mecanum)
    {
        MatrixXd m(4,3);
        m(0,0) = 1.0;
        m(0,1) = 1.0;
        m(0,2) = (b_ + a_ * (1.0 / tan(alpha_)));
        m(1,0) = -1.0;
        m(1,1) = 1.0;
        m(1,2) =  (b_ + a_ * (1.0 / tan(alpha_)));
        m(2,0) =  (1.0 / tan(alpha_));
        m(2,1) = -1.0;
        m(2,2) =  (b_ + a_ * (1.0 / tan(alpha_)));
        m(3,0) = 1.0;
        m(3,1) = -1.0;
        m(3,2) = (b_ + a_ * (1.0 / tan(alpha_)));

        w_mecanum =( m * v_base ) /r_;
    }

    void  Pi3Hat_Vel_Controller::homing_start_srv(const shared_ptr<TransactionService::Request> req, 
                                  const shared_ptr<TransactionService::Response> res)
    {
        {
            lock_guard<mutex> a(calbck_m_);
            RCLCPP_INFO(get_node()->get_logger(),"call homing service");
            if(state_ == Controller_State::PRE_HOMING && req->data)
            {
                homing_start_ = make_shared<rclcpp::Time>(get_node()->now());
                state_ = Controller_State::HOMING;
                res ->success = true;
                res ->message = string("Homing has been started");
                RCLCPP_INFO(get_node()->get_logger(),"OK");
            }
            else if(!req->data)
            {
                res->success = false;
                res->message = string("Request is not correct");
            }
            else if(state_ != Controller_State::PRE_HOMING)
            {
                res->success = false;
                res->message = string("Can not call the homing in this state");
            }
            RCLCPP_INFO(get_node()->get_logger(),"end homing service");
        }
    }

    void Pi3Hat_Vel_Controller::emergency_srv(const shared_ptr<TransactionService::Request> req, 
                                  const shared_ptr<TransactionService::Response> res)
    {
        
        std::lock_guard<std::mutex> a(calbck_m_);
        if(state_ != Controller_State::EMERGENCY && req->data)
        {
            state_ = Controller_State::EMERGENCY;
            for(auto &it:kp_scale_cmd_)
            {
                it.second = 0.0;
            }
            for(auto &it:kd_scale_cmd_)
            {
                it.second = 0.0;
            }
            res ->success = true;
            res ->message = string("Emergency mode has been activated");
        }
        else
        {
            res ->success = true;
            if(req->data)
                res ->message = string("The state is just in Emergency mode");
            else
                res -> message = string("Request is not correct");
        }

    }

    void Pi3Hat_Vel_Controller::compute_leg_joints_vel_ref(VectorXd& q_leg, VectorXd& q_dot_leg, LEG_IND l_index, double height_rate_tmp)
    {
        q_dot_leg(0) = 0.0;     //HAA always zero till now

        double s12 = sin(q_leg(0) + q_leg(1));
        double c12 = cos(q_leg(0) + q_leg(1));
        double den = c12 * (sin(q_leg(0) + s12)) - s12 * (cos(q_leg(0)) + c12); 
        
        q_dot_leg(1) = (1 / LINK_LENGHT) * (sin(q_leg(0)) + s12) / den * (- height_rate_tmp);      // height_rate is referred to the floating base while there we consider the foot velocity
        q_dot_leg(0) = - s12 / (sin(q_leg(0)) + s12) * q_dot_leg(0);                               // this is true until the foot remains under the hip 
        if(l_index == LEG_IND::RH || l_index == LEG_IND::LF)
        {
            q_dot_leg(1) *= -1.0;
            q_dot_leg(0) *= -1.0;
        }
    }

    void Pi3Hat_Vel_Controller::compute_homing_ref(LEG_IND l_i)
    {
        auto now = get_node()->now();
        double hip_val,knee_val,d_hip_val,d_knee_val;
        double dt_sec = now.seconds() - homing_start_->seconds();
        // RCLCPP_INFO(get_node()->get_logger(),"the time is %f",dt_sec);
        if(dt_sec <= homing_dur_)
        {
            hip_val = spline_par_[1]*dt_sec*dt_sec*dt_sec + spline_par_[0]*dt_sec*dt_sec;
            knee_val = spline_par_[3]*dt_sec*dt_sec*dt_sec + spline_par_[2]*dt_sec*dt_sec;
            d_hip_val = 3*spline_par_[1]*dt_sec*dt_sec + 2*spline_par_[0]*dt_sec;
            d_knee_val = 3*spline_par_[3]*dt_sec*dt_sec + 2*spline_par_[2]*dt_sec;
            if(l_i == LEG_IND::RF || l_i == LEG_IND::LH)
            {
                
                position_cmd_[joints_[2*l_i]] = hip_val;
                position_cmd_[joints_[2*l_i+1]] = knee_val;
                velocity_cmd_[joints_[2*l_i]] = d_hip_val;
                velocity_cmd_[joints_[2*l_i+1]] = d_knee_val;
            }      
            if(l_i == LEG_IND::RH || l_i == LEG_IND::LF )
            {
                position_cmd_[joints_[2*l_i]] = - hip_val;
                position_cmd_[joints_[2*l_i+1]] = - knee_val;
                velocity_cmd_[joints_[2*l_i]] = - d_hip_val;
                velocity_cmd_[joints_[2*l_i+1]] = - d_knee_val;
            }
        }
        else
        {
            if(l_i == LEG_IND::RF || l_i == LEG_IND::LH)
            {
                position_cmd_[joints_[2*l_i]] = RF_HFE_HOM;
                position_cmd_[joints_[2*l_i+1]] = RF_KFE_HOM;
                velocity_cmd_[joints_[2*l_i]] = 0.0;
                velocity_cmd_[joints_[2*l_i+1]] = 0.0;
            }
         

            if(l_i == LEG_IND::RH || l_i == LEG_IND::LF )
            {
                position_cmd_[joints_[2*l_i]] = -RF_HFE_HOM;
                position_cmd_[joints_[2*l_i+1]] = -RF_KFE_HOM;
                velocity_cmd_[joints_[2*l_i]] = 0.0;
                velocity_cmd_[joints_[2*l_i+1]] = 0.0;
            }
            // for (size_t i = LEG_NUM * JNT_LEG_NUM; i < LEG_NUM * JNT_LEG_NUM + WHL_NUM; i++)
            // {
            //     position_cmd_.at(joints_[i]) = 0.0;
            // }
            state_ = Controller_State::ACTIVE;

        }


    }

    void Pi3Hat_Vel_Controller::IK_RF(double &q1, double &q2, double y)
    {
        double c_a,s_a,a,c2,s2,c1,s1;
        c_a = 1- (std::pow(y,2))/(2* std::pow(LEG_LENGTH,2));
        s_a = std::sqrt(1 - std::pow(c_a,2));
        
        q2 = std::atan2(s_a,c_a);
        c2 = std::cos(q2);
        s2 = std::sin(q2);

        c1 = (y*s2)/(2*LEG_LENGTH*(1+c2));
        s1 = (y -s2*c1*LEG_LENGTH)/(LEG_LENGTH*(1+c2));
        q1 = std::atan2(s1,c1);

    };
 
    controller_interface::return_type Pi3Hat_Vel_Controller::update(const rclcpp::Time & , const rclcpp::Duration & dur)
    {
        std::string type;
        //floating base velocity
        double v_x, v_y, omega, height_rate;   
        // get seconds/ milliseconds from duration and compute DeltaT in second
        double deltaT = dur.seconds(); 

      
        for(auto &stt_int : state_interfaces_)
        {
            type = stt_int.get_interface_name();
            try
            {
                if(type == hardware_interface::HW_IF_POSITION)
                    position_out_.at(stt_int.get_prefix_name()) = stt_int.get_value();
                else if(type == hardware_interface::HW_IF_TEMPERATURE)
                    temperature_out_.at(stt_int.get_prefix_name()) = stt_int.get_value();
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"Interface name not correspond to the declared one");
                    return controller_interface::return_type::ERROR;
                }
            }
            catch(std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"catch error %s during the access to '%s' data",e.what(),stt_int.get_name().c_str());
            }
        }

        {
            lock_guard<mutex> a(calbck_m_);
            switch (state_)
            {
            case Controller_State::PRE_HOMING:

                break;
            case Controller_State::HOMING:
                compute_homing_ref(RF);
                compute_homing_ref(LF);
                compute_homing_ref(LH);
                compute_homing_ref(RH);
                
                break;
            case Controller_State::ACTIVE:
                //get the velocity target from the specific topic
                get_target(v_x, v_y, omega, height_rate);  

                //compute the joints reference from velocity target
                compute_reference(v_x, v_y, omega, height_rate, deltaT);
                break;
            default:
                break;
            }
        }
        

        for(auto &cmd_int : command_interfaces_)
        {
            type = cmd_int.get_interface_name();
            try
            {
                if(type == hardware_interface::HW_IF_POSITION)
                    cmd_int.set_value(position_cmd_.at(cmd_int.get_prefix_name()));
                else if(type == hardware_interface::HW_IF_VELOCITY)
                    cmd_int.set_value(velocity_cmd_.at(cmd_int.get_prefix_name()));
                else if(type == hardware_interface::HW_IF_EFFORT)
                    cmd_int.set_value(effort_cmd_.at(cmd_int.get_prefix_name()));
                else if(type == hardware_interface::HW_IF_KP_SCALE)
                    cmd_int.set_value(kp_scale_cmd_.at(cmd_int.get_prefix_name()));
                else if(type == hardware_interface::HW_IF_KD_SCALE)
                    cmd_int.set_value(kd_scale_cmd_.at(cmd_int.get_prefix_name()));
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"Interface name not correspond to the declared one");
                    return controller_interface::return_type::ERROR;
                }
            }
            catch(std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"catch error %s during the access to '%s' data",e.what(),cmd_int.get_name().c_str());
            }
        }
        #if DEBUG
            DebugMsgs msg_deb,w_msg;
            
            size_t msg_size = joints_.size() - 4;
            msg_deb.name.resize(msg_size);
            msg_deb.position.resize(msg_size);
            msg_deb.velocity.resize(msg_size);
            w_msg.name.resize(4);
            w_msg.position.resize(4);
            w_msg.velocity.resize(4);
            msg_deb.header.set__stamp(get_node()->now());
            for(size_t i = 0; i < msg_size; i++)
            {
                msg_deb.name[i] = joints_[i];
                msg_deb.position[i] = position_cmd_.at(joints_[i]);
                msg_deb.velocity[i] = velocity_cmd_.at(joints_[i]);
            }
            // for(auto & cmp : msg_deb.position)
            // {
            //     RCLCPP_INFO(get_node()->get_logger(),"the jnt has cmd value %f",cmp);
            // }
            pub_deb_->publish(msg_deb);

            for(size_t i = msg_size; i < joints_.size();i++)
            {
                // RCLCPP_INFO(get_node()->get_logger(),"%ld is the msg index and %ld is the jnt ind",i,i-msg_size);
                // RCLCPP_INFO(get_node()->get_logger(),"%s jnt ame ",joints_[i].c_str());
                // // RCLCPP_INFO(get_node()->get_logger(),"%",i,i-msg_size);
                w_msg.name[i-msg_size] = joints_[i];
                w_msg.position[i-msg_size] = position_cmd_.at(joints_[i]);
                w_msg.velocity[i-msg_size] = velocity_cmd_.at(joints_[i]);

            }
            wheel_pub_->publish(w_msg);


        #endif

        //  for(auto &cmd_int : command_interfaces_)
        // {
        //     type = cmd_int.get_interface_name();
        //     try
        //     {
        //         if(type == hardware_interface::HW_IF_POSITION)
        //             RCLCPP_INFO(rclcpp::get_logger("AAA"),"pos is %f",cmd_int.get_value());
                
        //         else
        //         {
        //             RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"Interface name not correspond to the declared one");
        //             return controller_interface::return_type::ERROR;
        //         }
        //     }
        //     catch(std::exception &e)
        //     {
        //         RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"catch error %s during the access to '%s' data",e.what(),cmd_int.get_name().c_str());
        //     }
        // }
 
        return controller_interface::return_type::OK;

    }
    
    controller_interface::InterfaceConfiguration Pi3Hat_Vel_Controller::state_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration stt_int_cnf;
        stt_int_cnf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for(auto &it : position_out_)
        {
            stt_int_cnf.names.push_back(it.first + "/" + hardware_interface::HW_IF_POSITION);
            stt_int_cnf.names.push_back(it.first + "/" + hardware_interface::HW_IF_TEMPERATURE);
        }
        return stt_int_cnf;
    }

    controller_interface::InterfaceConfiguration Pi3Hat_Vel_Controller::command_interface_configuration() const
    {
        controller_interface::InterfaceConfiguration cmd_int_cnf;
        cmd_int_cnf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for(auto &it : position_cmd_)
        {
            cmd_int_cnf.names.push_back(it.first + "/" + hardware_interface::HW_IF_POSITION);
            cmd_int_cnf.names.push_back(it.first + "/" + hardware_interface::HW_IF_VELOCITY);
            cmd_int_cnf.names.push_back(it.first + "/" + hardware_interface::HW_IF_EFFORT);
            cmd_int_cnf.names.push_back(it.first + "/" + hardware_interface::HW_IF_KP_SCALE);
            cmd_int_cnf.names.push_back(it.first + "/" + hardware_interface::HW_IF_KD_SCALE);
        }
        return cmd_int_cnf;
    }
};

PLUGINLIB_EXPORT_CLASS(
    pi3hat_vel_controller::Pi3Hat_Vel_Controller, controller_interface::ControllerInterface
);
