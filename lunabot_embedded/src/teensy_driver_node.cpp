#include <ros/ros.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include <sys/ioctl.h>
#include <termios.h>

#include <lunabot_msgs/RobotEffort.h>
#include <lunabot_msgs/RobotState.h>

extern "C" {
#include "RobotMsgs.pb.h"
#include "hid.h"
#include "pb_decode.h"
#include "pb_encode.h"
}

#define BUF_SIZE 64

uint8_t buf[BUF_SIZE];

RobotState state = RobotState_init_zero;
RobotEffort effort = RobotEffort_init_zero;

pb_ostream_t sizestream = {0};

void recv(ros::Publisher &pub) {
    int status;
    /* Create a stream that reads from the buffer. */
    pb_istream_t stream = pb_istream_from_buffer(buf, sizeof(buf));
    /* Now we are ready to decode the message. */
    pb_decode(&stream, RobotState_fields, &state);
    lunabot_msgs::RobotState state_msg;
    state_msg.act_right_curr = state.act_right_curr;
    state_msg.drive_right_curr = state.drive_right_curr;
    state_msg.drive_left_curr = state.drive_left_curr;
    state_msg.lead_screw_curr = state.lead_screw_curr;
    state_msg.dep_curr = state.dep_curr;
    state_msg.exc_curr = state.exc_curr;
    state_msg.act_ang = state.act_ang;
    state_msg.drive_right_ang = state.drive_right_ang;
    state_msg.drive_left_ang = state.drive_left_ang;
    state_msg.lead_screw_ang = state.lead_screw_ang;
    state_msg.dep_ang = state.dep_ang;
    pub.publish(state_msg);
}

void effort_cb(const lunabot_msgs::RobotEffort &msg) {
    effort.lead_screw = msg.lead_screw;
    effort.lin_act = msg.lin_act;
    effort.left_drive = msg.left_drive;
    effort.right_drive = msg.right_drive;
    effort.excavate = msg.excavate;
    effort.deposit = msg.deposit;
}

void publish(const ros::TimerEvent&) {
        memset(buf, 0, sizeof(buf));
        pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
        pb_encode(&stream, RobotEffort_fields, &effort);
    	rawhid_send(0, buf, 64, 0);
}

int main(int argc, char **argv) {

    int i, r, num;
    r = rawhid_open(1, 0x16C0, 0x0486, 0xFFAB, 0x0200);
    if (r <= 0) {
        printf("no rawhid device found\n");
        return -1;
    }
    printf("found rawhid device\n");

    ros::init(argc, argv, "teensy_driver_node");
    ros::NodeHandle nh;

    ros::Subscriber effort_sub = nh.subscribe("/effort", 10, &effort_cb);
    ros::Publisher state_pub =
        nh.advertise<lunabot_msgs::RobotState>("/state", 10);

    ros::Rate rate(100);

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), publish);

    while (ros::ok()) {
        // check if any Raw HID packet has arrived
        ros::spinOnce();
        num = rawhid_recv(0, buf, BUF_SIZE, 0);
        if (num < 0) {
            printf("\nerror reading, device went offline\n");
            break;
        }

        if (num > 0) {
            recv(state_pub);
        }

        rate.sleep();
    }
    rawhid_close(0);
}