using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Tf2;
using System;
using System.Collections.Generic;

public class CmdVelSubscriber : MonoBehaviour
{
    public ArticulationBody frontLeftWheel;
    public ArticulationBody frontRightWheel;
    public ArticulationBody backLeftWheel;
    public ArticulationBody backRightWheel;

    public float wheelRadius = 0.127f;
    public float robotLenght = 0.573f;

    private float leftSpeed = 0f;
    private float rightSpeed = 0f;

    private ROSConnection rosConnection;
    private string odomTopic = "/odom";
    private string tfTopic = "/tf";
    private string pathTopic = "/path";

    private Vector3 position = Vector3.zero;
    private float theta = 0f;

    private PathMsg pathMsg;

    void Start()
    {
        rosConnection = ROSConnection.GetOrCreateInstance();
        rosConnection.Subscribe<TwistMsg>("/cmd_vel", ApplyVelocity);
        rosConnection.RegisterPublisher<OdometryMsg>(odomTopic);
        rosConnection.RegisterPublisher<TFMessageMsg>(tfTopic);
        rosConnection.RegisterPublisher<PathMsg>(pathTopic);

        pathMsg = new PathMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                frame_id = "odom"
            },
            poses = new PoseStampedMsg[] { }
        };
    }

    void Update()
    {
        float dt = Time.deltaTime;
        ApplySpeeds();
        PublishOdometry(dt);
    }

    void ApplyVelocity(TwistMsg msg)
    {
        float vx = (float)msg.linear.x;
        float wz = (float)msg.angular.z;

        float v_left = vx - (robotLenght / 2f) * wz;
        float v_right = vx + (robotLenght / 2f) * wz;

        leftSpeed = v_left / wheelRadius;
        rightSpeed = v_right / wheelRadius;
    }

    void ApplySpeeds()
    {
        SetWheelSpeed(frontLeftWheel, leftSpeed);
        SetWheelSpeed(backLeftWheel, leftSpeed);
        SetWheelSpeed(frontRightWheel, rightSpeed);
        SetWheelSpeed(backRightWheel, rightSpeed);
    }

    void SetWheelSpeed(ArticulationBody wheel, float speed)
    {
        var drive = wheel.xDrive;
        drive.targetVelocity = speed * Mathf.Rad2Deg;
        drive.forceLimit = 1000f;
        wheel.xDrive = drive;
    }

    void PublishOdometry(float dt)
    {
        float v = (leftSpeed + rightSpeed) * wheelRadius / 2f;
        float w = (rightSpeed - leftSpeed) * wheelRadius / robotLenght;

        float deltaTheta = w * dt;
        theta += deltaTheta;
        float dx = v * Mathf.Cos(theta) * dt;
        float dz = v * Mathf.Sin(theta) * dt;
        position += new Vector3(dx, 0, dz);

        var timeStamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg((uint)Time.realtimeSinceStartup, 0);

        OdometryMsg odom = new OdometryMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = timeStamp,
                frame_id = "robot_base_footprint"
            },
            child_frame_id = "robot_base_link",
            pose = new PoseWithCovarianceMsg
            {
                pose = new PoseMsg
                {
                    position = new PointMsg(position.x, 0, position.z),
                    orientation = QuaternionToMsg(Quaternion.Euler(0, theta * Mathf.Rad2Deg, 0))
                }
            },
            twist = new TwistWithCovarianceMsg
            {
                twist = new TwistMsg
                {
                    linear = new Vector3Msg(v, 0, 0),
                    angular = new Vector3Msg(0, w, 0)
                }
            }
        };
        rosConnection.Publish(odomTopic, odom);

        TransformStampedMsg tf = new TransformStampedMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = timeStamp,
                frame_id = "robot_base_footprint"
            },
            child_frame_id = "robot_base_link",
            transform = new TransformMsg
            {
                translation = new Vector3Msg(position.x, 0, position.z),
                rotation = QuaternionToMsg(Quaternion.Euler(0, theta * Mathf.Rad2Deg, 0))
            }
        };
        rosConnection.Publish(tfTopic, new TFMessageMsg(new TransformStampedMsg[] { tf }));

        PoseStampedMsg poseStamped = new PoseStampedMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = timeStamp,
                frame_id = "robot_base_footprint"
            },
            pose = new PoseMsg
            {
                position = new PointMsg(position.x, 0, position.z),
                orientation = QuaternionToMsg(Quaternion.Euler(0, theta * Mathf.Rad2Deg, 0))
            }
        };

        var poses = new List<PoseStampedMsg>(pathMsg.poses);
        poses.Add(poseStamped);
        pathMsg.poses = poses.ToArray();
        pathMsg.header.stamp = timeStamp;
        rosConnection.Publish(pathTopic, pathMsg);
    }

    QuaternionMsg QuaternionToMsg(Quaternion q)
    {
        return new QuaternionMsg(q.x, q.y, q.z, q.w);
    }
}

