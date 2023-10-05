#include "pluginlib/class_list_macros.hpp"
#include "pi3hat_omni_controller/pi3hat_vel_controller.hpp"
#include "pi3hat_hw_interface/motor_manager.hpp"
#include "std_srvs/srv/"
#include <cstdint>

namespace pi3hat_vel_controller
{

    
    Pi3Hat_Vel_Controller::Pi3Hat_Vel_Controller():
    cmd_sub_(nullptr),
    logger_name_("Pi3Hat_Vel_Controller"),
    //rt_buffer_(nullptr),
    vel_target_rcvd_msg_(nullptr),
    default_init_pos_(false)
    //prova
    {}

    Pi3Hat_Vel_Controller::~Pi3Hat_Vel_Controller()
    {
        // rt_buffer_.~RealtimeBuffer();
    }

    CallbackReturn Pi3Hat_Vel_Controller::on_init()
    {


        // declare homing duration params
        try
        {
            auto_declare<double>("homing_duration",5.0);         
        }
         catch(const std::exception & e)
        {
            RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"Exception thrown during declaration of joints name with message: %s", e.what());
            return CallbackReturn::ERROR;
        }

        // da qui 
        try
        {
            auto_declare<std::vector<std::string>>("joints",std::vector<std::string>());           //sto dichiarando le interfacce
        }
         catch(const std::exception & e)
        {
            RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"Exception thrown during declaration of joints name with message: %s", e.what());
            return CallbackReturn::ERROR;
        }
        
        // a qui tolto 
        try
        {
            auto_declare<std::vector<double>>("init_pos",std::vector<double>());                   //sto dichiarando le posizioni inniziali
        }
          catch(const std::exception & e)
        {
            RCLCPP_WARN(rclcpp::get_logger(logger_name_),"Exception thrown during declaretion of init position with message: %s it gets default values", e.what());
            
        }
        default_init_pos_ = true;
        vel_target_rcvd_msg_ = std::make_shared<CmdMsgs>();
        RCLCPP_INFO(get_node()->get_logger(),"initialize succesfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn Pi3Hat_Vel_Controller::on_configure(const rclcpp_lifecycle::State &)
    {
        std::vector<double> init_positions;
        double homig_dur;
        size_t sz;
        // get the controlled joints name
        a_ = get_node()->get_parameter("driveshaft_y").as_double();
        b_ = get_node()->get_parameter("driveshaft_x").as_double();
        alpha_ = get_node()->get_parameter("mecanum_angle").as_double();

        //get homing duration 

        homing_dur_ = this->get_parameter("homing_duration").as_double();
        // set spline parameters
        spline_par_[0] = 3 * RF_HFE_HOM / (homing_dur_*homing_dur_); // a_2_hip
        spline_par_[1] = -2 * RF_HFE_HOM /( homing_dur_*homing_dur_*homing_dur_); // a_3_hip
        spline_par_[2] = 3 * RF_KFE_HOM / (homing_dur_*homing_dur_); // a_2_knee
        spline_par_[3] = -2 * RF_KFE_HOM /( homing_dur_*homing_dur_*homing_dur_); // a_3_knee

        // a questo punto si può aggiungiungere joints_ come membro della classe e settarlo nell'hpp
        //std::vector<std::string> joint = get_node()->get_parameter("joints").as_string_array();

        if(joint.empty())
        {
            RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"'joints' parameter is empty");
            return CallbackReturn::ERROR;
        }
        
        // init the initial position if its needed
        if(!default_init_pos_)
        {
            init_positions = get_node()->get_parameter("init_pos").as_double_array();
            if(init_positions.empty())
            {
                RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"'init_pos' parameter is empty");
                return CallbackReturn::ERROR;
            }
            if(init_positions.size() != joint.size())
            {
                RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"'init_pos' and 'joints' can not have different dimension");
                return CallbackReturn::ERROR;
            }
        }
        else
            init_positions.resize(joint.size(),0.0);
        
        // fill the map structure 
        sz = joint.size();
        for(size_t i = 0; i < sz; i++)
        {
            // da mettere joints_ al posto di joint nuovo membro 
            position_cmd_.emplace(std::make_pair(joint[i],init_positions[i])); // init with NaN
            position_out_.emplace(std::make_pair(joint[i],init_positions[i]));                //used to store current measured joint position
            temperature_out_.emplace(std::make_pair(joint[i],init_positions[i]));             //used to store current measured joint temperature
            velocity_cmd_.emplace(std::make_pair(joint[i],0.0));
            effort_cmd_.emplace(std::make_pair(joint[i],0.0));
            kp_scale_cmd_.emplace(std::make_pair(joint[i],1.0)); 
            kd_scale_cmd_.emplace(std::make_pair(joint[i],1.0));

        }

        // build the subscriber
        // ci pensa jacopino Add QOS and deadline event callback
        cmd_sub_ = get_node()->create_subscription<CmdMsgs>(
            "~/command",
            5,
            [this](const CmdMsgs::SharedPtr msg)
            {
                // rt_buffer_.writeFromNonRT(msg);
                // aggiungere il lock_guard std::lock_guard(<mutex_var>)
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
        if(vel_target_rcvd_msg_.get())
        {
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
        else
            RCLCPP_WARN(rclcpp::get_logger(logger_name_),"No data are readed from realtime buffer");
            
        // maybe needed?
        //joints_rcvd_msg_.reset();
    }

    bool Pi3Hat_Vel_Controller::compute_reference(double v_x_tmp, double v_y_tmp, double omega_tmp, double height_rate_tmp)// add duration as argument [s]
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
                velocity_cmd_.at(joint[i]) = w_mecanum[i - JNT_NUM];
                
            }
            catch(const std::exception& e)
            {
                RCLCPP_ERROR( rclcpp::get_logger(logger_name_),"Raised error during the wheel velocity references assegnation %s", e.what());
                return false;
            }            
        }
        
        for (size_t i = 0; i < LEG_NUM; i++)
        {   
            VectorXd q_leg(3), q_dot_leg(3);   //[HAA, HFE, KFE]
            
            //extract the i-th leg joint position info
            for (size_t j = 0; j < JNT_LEG_NUM; j++)
            {
                q_leg(j) = position_out_.at(joint[JNT_LEG_NUM*i + j]) ;           
            }
            
            compute_leg_joints_vel_ref(q_leg, q_dot_leg, i, height_rate_tmp);
            
            //insert the i-th leg joint velocity reference
            for (size_t j = 0; j < JNT_LEG_NUM; j++)
            {
                try
                {   
                    // add integration of pos with computed command vel, so we can send also position reference
                    velocity_cmd_.at(joint[JNT_LEG_NUM*i + j]) = q_dot_leg(j) 
                }
                catch(const std::exception& e)
                {
                    RCLCPP_ERROR( rclcpp::get_logger(logger_name_),"Raised error during the %d-th leg %d-th joint velocity references assegnation %s", i, j, e.what());
                    return false;
                }              
            }
        }
        

        return true;
    }

    void compute_mecanum_speed(VectorXd& v_base, VectorXd& w_mecanum)
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

    void compute_leg_joints_vel_ref(VectorXd& q_leg, VectorXd& q_dot_leg, size_t l_index, double height_rate_tmp)
    {
        q_dot_leg(0) = 0.0;     //HAA always zero till now

        double s12 = sin(q_leg(1) + q_leg(2));
        double c12 = cos(q_leg(1) + q_leg(2));
        double den = c12 * (sin(q_leg(1) + s12)) - s12 * (cos(q_leg(1)) + c12); 

        q_dot_leg(2) = (1 / LINK_LENGHT) * (sin(q_leg(1)) + s12) / den * (- height_rate_tmp);      // height_rate is referred to the floating base while there we consider the foot velocity
        q_dot_leg(1) = - s12 / (sin(q_leg(1)) + s12) * q_dot_leg(2);                               // this is true until the foot remains under the hip
    }

    void compute_homing_ref(LEG_IND l_i)
    {
        rclcpp::Time dt = this->now() - homing_start_;
        double dt_sec = dt.seconds();
        if(dt_sec <= homing_dur_)
        {
            if(l_i == LEG_IND::RF || l_i == LEG_IND::LH )
            {
                position_cmd_[joints_[3*l_i+1]] = spline_par_[1]*dt_sec*dt_sec*dt_sec + spline_par_[0]*dt_sec*dt_sec;
                position_cmd_[joints_[3*l_i+2]] = spline_par_[3]*dt_sec*dt_sec*dt_sec + spline_par_[2]*dt_sec*dt_sec;
                velocity_cmd_[joints_[3*l_i+1]] = 3*spline_par_[1]*dt_sec*dt_sec + 2*spline_par_[0]*dt_sec;
                velocity_cmd_[joints_[3*l_i+2]] = 3*spline_par_[3]*dt_sec*dt_sec + 2*spline_par_[2]dt_sec;
            }

            if(l_i == LEG_IND::RH || l_i == LEG_IND::LF )
            {
                position_cmd_[joints_[3*l_i+1]] = - spline_par_[1]*dt_sec*dt_sec*dt_sec + spline_par_[0]*dt_sec*dt_sec;
                position_cmd_[joints_[3*l_i+2]] = - spline_par_[3]*dt_sec*dt_sec*dt_sec + spline_par_[2]*dt_sec*dt_sec;
                velocity_cmd_[joints_[3*l_i+1]] = - 3*spline_par_[1]*dt_sec*dt_sec + 2*spline_par_[0]*dt_sec;
                velocity_cmd_[joints_[3*l_i+2]] = - 3*spline_par_[3]*dt_sec*dt_sec + 2*spline_par_[2]dt_sec;
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


    controller_interface::return_type Pi3Hat_Vel_Controller::update(const rclcpp::Time & , const rclcpp::Duration & dur)
    {
        std::string type;
        double v_x, v_y, omega, height_rate;   
        // get seconds/ milliseconds from duration and compute DeltaT in second
                  //floating base velocity
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
                RCLCPP_ERROR(rclcpp::get_logger(logger_name_),"catch error %s during the access to '%s' data",e.what(),cmd_int.get_name().c_str());
            }
        }


        switch (state_)
        {
        case Controller_State::INACTIVE:
            compute_homing_ref(RF);
            compute_homing_ref(LF);
            compute_homing_ref(LH);
            compute_homing_ref(RH);
            break;
        case Controller_State::ACTIVE
            //get the velocity target from the specific topic
            get_target(v_x, v_y, omega, height_rate);  

            //compute the joints reference from velocity target
            compute_reference(v_x, v_y, omega, height_rate);
            break;
        default:
            break;
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
                    cmd_int.set_value(velocity_cmd_.at(cmd_in`t.get_prefix_name()));
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
    pi3hat_Vel_controller::Pi3Hat_Vel_Controller, controller_interface::ControllerInterface
);
