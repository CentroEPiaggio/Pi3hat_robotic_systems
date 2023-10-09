#include "pluginlib/class_list_macros.hpp"
#include "pi3hat_omni_controller/pi3hat_vel_controller.hpp"
#include "pi3hat_hw_interface/motor_manager.hpp"
#include <cstdint>
#include "eigen3/Eigen/Core"
#define VEL_CMD true
#define MAX_LOSS 20

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
        double homig_dur;
        size_t sz;
        // get the controlled joints name
        a_ = get_node()->get_parameter("driveshaft_y").as_double();
        b_ = get_node()->get_parameter("driveshaft_x").as_double();
        alpha_ = get_node()->get_parameter("mecanum_angle").as_double();

        std::chrono::duration dur = std::chrono::milliseconds(get_node()->get_parameter("input_frequency").as_int());

        //get homing duration 

        homing_dur_ = get_node()->get_parameter("homing_duration").as_double();
        // set spline parameters
        spline_par_[0] = 3 * RF_HFE_HOM / (homing_dur_*homing_dur_); // a_2_hip
        spline_par_[1] = -2 * RF_HFE_HOM /( homing_dur_*homing_dur_*homing_dur_); // a_3_hip
        spline_par_[2] = 3 * RF_KFE_HOM / (homing_dur_*homing_dur_); // a_2_knee
        spline_par_[3] = -2 * RF_KFE_HOM /( homing_dur_*homing_dur_*homing_dur_); // a_3_knee
        
        // fill the map structure 
        sz = joints_.size();
        for(size_t i = 0; i < sz; i++)
        {
            // da mettere joints_ al posto di joint nuovo membro 
            position_cmd_.emplace(std::make_pair(joints_[i],i<(JNT_LEG_NUM)*LEG_NUM?0.0:std::nan("0"))); // init with NaN
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
        
        cmd_sub_ = get_node()->create_subscription<CmdMsgs>(
            "~/command",
            5,
            [this](const CmdMsgs::SharedPtr msg)
            {
                // rt_buffer_.writeFromNonRT(msg);
                
                // aggiungere il lock_guard std::lock_guard(<mutex_var_>)
                lock_guard<std::mutex> lock(mutex_var_);
                loss_counter_ = 0;
                vel_target_rcvd_msg_->set__v_x(msg->v_x);
                vel_target_rcvd_msg_->set__v_y(msg->v_y);
                vel_target_rcvd_msg_->set__omega(msg->omega);
                vel_target_rcvd_msg_->set__height_rate(msg->height_rate);
            }
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
        
            
    }

    bool Pi3Hat_Vel_Controller::compute_reference(double v_x_tmp, double v_y_tmp, double omega_tmp, double height_rate_tmp, double dt)// add duration as argument [s]
    {   
        VectorXd v_base(3);
        VectorXd w_mecanum(4);
        v_base << v_x_tmp, v_y_tmp, omega_tmp;

        compute_mecanum_speed(v_base, w_mecanum);

        //update wheels_velocity_cmd map, last four elements of the joint list
        for (size_t i = LEG_NUM * JNT_LEG_NUM; i < LEG_NUM * JNT_LEG_NUM + WHL_NUM; i++)
        {
            try
            {   
                #if VEL_CCM 
                    position_cmd_.at(joints_[i]) = std::nan();
                #else
                    position_cmd_.at(joints_[i]) += dt* velocity_cmd_.at(joints_[i]);
                #endif 
                velocity_cmd_.at(joints_[i]) = w_mecanum[i - JNT_LEG_NUM * LEG_NUM];
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR( rclcpp::get_logger(logger_name_),"Raised error during the wheel velocity references assegnation %s", e.what());
                return false;
            }            
        }
        
        for (auto &i:legs_)
        {   
            VectorXd q_leg(3), q_dot_leg(3);   //[HAA, HFE, KFE]
            
            //extract the i-th leg joint position info
            for (size_t j = 0; j < JNT_LEG_NUM; j++)
            {
                q_leg(j) = position_out_.at(joints_[JNT_LEG_NUM*i + j]) ;           
            }
            
            compute_leg_joints_vel_ref(q_leg, q_dot_leg, i, height_rate_tmp);
            
            //insert the i-th leg joint velocity reference
            for (size_t j = 0; j < JNT_LEG_NUM; j++)
            {
                try
                {   
                    // add integration of pos with computed command vel, so we can send also position reference  
                    position_cmd_.at(joints_[JNT_LEG_NUM*i + j]) = q_dot_leg(j) * dt + q_leg(j);
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
        m(0,0) = -(1.0 / tan(alpha_));
        m(0,1) = 1.0;
        m(0,2) = (b_ + a_ * (1 / tan(alpha_)));
        m(1,0) = (1.0 / tan(alpha_));
        m(1,1) = 1.0;
        m(1,2) = - (b_ + a_ * (1 / tan(alpha_)));
        m(2,0) = - (1.0 / tan(alpha_));
        m(2,1) = 1.0;
        m(2,2) = - (b_ + a_ * (1 / tan(alpha_)));
        m(3,0) = (1.0 / tan(alpha_));
        m(3,1) = 1.0;
        m(3,2) = (b_ + a_ * (1 / tan(alpha_)));

        w_mecanum = m * v_base;
    }

    void  Pi3Hat_Vel_Controller::homing_start_srv(const shared_ptr<TransactionService::Request> req, 
                                  const shared_ptr<TransactionService::Response> res)
    {
        lock_guard<mutex> a(calbck_m_);
        if(state_ == Controller_State::PRE_HOMING && req->data)
        {
            homing_start_ = make_shared<rclcpp::Time>(get_node()->now());
            state_ = Controller_State::HOMING;
            res ->success = true;
            res ->message = string("Homing has been started");
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

        double s12 = sin(q_leg(1) + q_leg(2));
        double c12 = cos(q_leg(1) + q_leg(2));
        double den = c12 * (sin(q_leg(1) + s12)) - s12 * (cos(q_leg(1)) + c12); 
        
        if(l_index == LEG_IND::RF || l_index == LEG_IND::LH)
        {
            q_dot_leg(2) = (1 / LINK_LENGHT) * (sin(q_leg(1)) + s12) / den * (- height_rate_tmp);      // height_rate is referred to the floating base while there we consider the foot velocity
            q_dot_leg(1) = - s12 / (sin(q_leg(1)) + s12) * q_dot_leg(2);                               // this is true until the foot remains under the hip
        }
        else
        {
            q_dot_leg(2) = - (1 / LINK_LENGHT) * (sin(q_leg(1)) + s12) / den * (- height_rate_tmp);    
            q_dot_leg(1) =  s12 / (sin(q_leg(1)) + s12) * q_dot_leg(2);
        }
    }

    void Pi3Hat_Vel_Controller::compute_homing_ref(LEG_IND l_i)
    {
        auto now = get_node()->now();
       
        double dt_sec = now.seconds() - homing_start_->seconds();
        if(dt_sec <= homing_dur_)
        {
            if(l_i == LEG_IND::RF || l_i == LEG_IND::LH )
            {
                position_cmd_[joints_[3*l_i+1]] = spline_par_[1]*dt_sec*dt_sec*dt_sec + spline_par_[0]*dt_sec*dt_sec;
                position_cmd_[joints_[3*l_i+2]] = spline_par_[3]*dt_sec*dt_sec*dt_sec + spline_par_[2]*dt_sec*dt_sec;
                velocity_cmd_[joints_[3*l_i+1]] = 3*spline_par_[1]*dt_sec*dt_sec + 2*spline_par_[0]*dt_sec;
                velocity_cmd_[joints_[3*l_i+2]] = 3*spline_par_[3]*dt_sec*dt_sec + 2*spline_par_[2]*dt_sec;
            }

            if(l_i == LEG_IND::RH || l_i == LEG_IND::LF )
            {
                position_cmd_[joints_[3*l_i+1]] = - spline_par_[1]*dt_sec*dt_sec*dt_sec + spline_par_[0]*dt_sec*dt_sec;
                position_cmd_[joints_[3*l_i+2]] = - spline_par_[3]*dt_sec*dt_sec*dt_sec + spline_par_[2]*dt_sec*dt_sec;
                velocity_cmd_[joints_[3*l_i+1]] = - 3*spline_par_[1]*dt_sec*dt_sec + 2*spline_par_[0]*dt_sec;
                velocity_cmd_[joints_[3*l_i+2]] = - 3*spline_par_[3]*dt_sec*dt_sec + 2*spline_par_[2]*dt_sec;
            }
        }
        else
        {
            if(l_i == LEG_IND::RF || l_i == LEG_IND::LH )
            {
                position_cmd_[joints_[3*l_i+1]] = RF_HFE_HOM;
                position_cmd_[joints_[3*l_i+2]] = RF_KFE_HOM;
                velocity_cmd_[joints_[3*l_i+1]] = 0.0;
                velocity_cmd_[joints_[3*l_i+2]] = 0.0;
            }

            if(l_i == LEG_IND::RH || l_i == LEG_IND::LF )
            {
                position_cmd_[joints_[3*l_i+1]] = -RF_HFE_HOM;
                position_cmd_[joints_[3*l_i+2]] = -RF_KFE_HOM;
                velocity_cmd_[joints_[3*l_i+1]] = 0.0;
                velocity_cmd_[joints_[3*l_i+2]] = 0.0;
            }
            state_ = Controller_State::ACTIVE;

        }


    }

    // double Pi3Hat_Vel_Controller::duration_to_s(rclcpp::Duration d)
    //     {
    //     long  ns = d.nanoseconds();
    //     double tot_s = (double)ns/1000000000.0;      //bruttissima ma funge
    //     return tot_s;
    //     }

    controller_interface::return_type Pi3Hat_Vel_Controller::update(const rclcpp::Time & , const rclcpp::Duration & dur)
    {
        std::string type;
        //floating base velocity
        double v_x, v_y, omega, height_rate;   
        // get seconds/ milliseconds from duration and compute DeltaT in second
        double deltaT = dur.seconds(); 

        // set the data from the readed message
        // if(!get_reference())
        //     return controller_interface::return_type::ERROR;
        //get_reference();
        
        // get the state variables iterating over the exported state interface
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
        


        // set the commanded reference iterating over the exported commanded interface
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
