//class to have a node that can simultaneously subscribe and publish on two different topics
#ifndef ROBOTICS_SUBPUB_H
#define ROBOTICS_SUBPUB_H

#include "ros/ros.h"
#include "string"

template<typename SubscribeT,typename PublishT> //type of the subscribed message, type of the published message
class SubPub{
public:
    SubPub() {}
    SubPub(std::string SubscribeTopicName,std::string PublishTopicName, int queueSize){
        subscriberObject=n.subscribe<SubscribeT>(SubscribeTopicName,queueSize,&SubPub::subscriberCallback,this);
        publisherObject=n.advertise<PublishT>(PublishTopicName,queueSize);
    }
    void subscriberCallback(const typename SubscribeT::ConstPtr& recievedMsg);

protected:
    ros::Subscriber subscriberObject;
    ros::Publisher publisherObject;
    ros::NodeHandle n;
};

#endif //ROBOTICS_SUBPUB_H
