#include <mutex>
#include "ca_gazebo/traffic_light_plugin.h"
#include <gazebo/common/Color.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/transport/Node.hh>
 #include <ignition/math/Color.hh>

namespace gazebo
{

    GZ_REGISTER_VISUAL_PLUGIN(GazeboTrafficLight)

    class GazeboTrafficLightPrivate
    {
        /// Visual whose color will be changed.
        public: rendering::VisualPtr visual;

        /// Connects to rendering update event.
        public: event::ConnectionPtr updateConnection;

        /// Time on for every color.
        public: common::Time red_time_    = common::Time(1);
        public: common::Time yellow_time_ = common::Time(1);
        public: common::Time green_time_  = common::Time(1);

        /// Time the current cycle started.
        public: common::Time cycleStartTime;

        /// The current simulation time.
        public: common::Time currentSimTime;

        /// Node used for communication.
        public: transport::NodePtr node;

        /// Node used for communication.
        public: std::mutex mutex;

        /// True to use wall time, false to use sim time.
        public: bool useWallTime;

        /// Subscriber to world info.
        public: transport::SubscriberPtr infoSub;
    };

    GazeboTrafficLight::GazeboTrafficLight() : data_ptr(new GazeboTrafficLightPrivate)
    {}

    GazeboTrafficLight::~GazeboTrafficLight()
    {}

    void GazeboTrafficLight::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
    {
        // Check for null pointer 
        if (!_visual || !_sdf)
        {
        gzerr << "No visual or SDF element specified. Plugin won't load." <<
            std::endl;
        return;
        }
        // Get parameters
        this->data_ptr->visual = _visual;
        /*this->data_ptr->red_time_    = _sdf-> Get<common::Time>("redTime");
        this->data_ptr->yellow_time_ = _sdf-> Get<common::Time>("yellowTime");
        this->data_ptr->green_time_  = _sdf-> Get<common::Time>("greenTime");*/
        // Check for correct time for every color
        if (this->data_ptr->red_time_ <= 0 || this->data_ptr->yellow_time_ <= 0 || this->data_ptr->green_time_ <= 0)
        {
        gzerr << "Time can't be lower than zero." << std::endl;
        return;
        }
        // Connect to the world update signal
        this->data_ptr->updateConnection = event::Events::ConnectPreRender(std::bind(&GazeboTrafficLight::Update, this));
        // Subscribe to world statistics to get sim time
        // Warning: topic ~/pose/local/info is meant for high-bandwidth local
        // network access. It will kill the system if a remote gzclient tries to
        // subscribe.
        // Create our node for communication
        this->data_ptr->node = transport::NodePtr(new transport::Node());
        // Initialize the node with the world name
        this->data_ptr->node->Init();
        // Listen to Gazebo world_stats topic
        commandSubscriber = this->data_ptr->node->Subscribe("/gazebo/EmptyWorld/WorldTime_topic", &GazeboTrafficLight::time_update_cb, this);
        this->next_color_ = "red";
    }

    void GazeboTrafficLight::Update()
    {
        //std::lock_guard<std::mutex> lock(this->data_ptr->mutex);

        if (!this->data_ptr->visual)
        {
            gzerr << "The visual is null." << std::endl;
            return;
        }

        common::Time period_time;

        if(this->next_color_ == "green")
            period_time = this->data_ptr->red_time_;
        else if(this->next_color_ == "red")
            period_time = this->data_ptr->yellow_time_;
        else
            period_time = this->data_ptr->green_time_;

        auto elapsed = this->current_time_ - this->last_time_;

        if(elapsed > period_time)
        {

            this->last_time_ = this->current_time_;

            double red = 0;
            double green = 0;
            double blue = 0;

            if(this->next_color_ == "red")
            {
                red = 1;
                this->next_color_ = "yellow";
            }
            else if(this->next_color_ == "yellow")
            {
                red = 1;
                green = 1;
                this->next_color_ = "green";
            }
            else
            {
                green = 1;
                this->next_color_ = "red";
            }

            ignition::math::Color color(red, green, blue);

            this->data_ptr->visual->SetDiffuse(color);
            this->data_ptr->visual->SetAmbient(color);
            this->data_ptr->visual->SetTransparency(1);
        }
    }

    void GazeboTrafficLight::time_update_cb(ConstTimePtr &_msg)
    {
        this->current_time_ = _msg->sec();
    }

}

