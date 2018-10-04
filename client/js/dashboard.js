var ros;
var scl_reach;

var viewer;

var user_arrow;
var ensta_arrow;
var last_user_arrow = new Date(); // Time of last valid reception.
var robot_pose;

function init() {
    ros = new ROSLIB.Ros({url: 'ws://' + window.location.hostname + ':9090'});

    init_hbba();
    var w = window,
    d = document,
    e = d.documentElement,
    g = d.getElementsByTagName('body')[0],
    x = w.innerWidth || e.clientWidth || g.clientWidth,
    y = w.innerHeight|| e.clientHeight|| g.clientHeight;

    viewer = new ROS2D.Viewer({
        divID: 'nav',
        width: x,
        height: y
    });

    var nav = NAV2D.OccupancyGridClientNav({
        ros:        ros,
        rootObject: viewer.scene,
        viewer:     viewer,
        serverName: '/motv_identification_manager/nav_to_pose'
    });

    


    var enstaText = new createjs.Text("ENSTA STAND", "2px Arial", "#ff7700");
    enstaText.textBaseline = "alphabetic";
    ensta_arrow = new ROS2D.NavigationArrow({
        size: 1.0,
        strokeColor: createjs.Graphics.getRGB(0, 0, 255, 0.5) 
    });
    ensta_arrow.scaleX = 0.1;
    ensta_arrow.scaleY = 0.1;


    glonalPlannerService = new ROSLIB.Service({
        ros:            ros,
        name:           '/move_base/GlobalPlanner/make_plan',
        serviceType:    'nav_msgs/GetPlan'
    });

    var pose_topic = new ROSLIB.Topic({
        ros:            ros,
        name:           '/robot_pose_web',
        messageType:    'geometry_msgs/Pose'
    });
    pose_topic.subscribe(function(msg) {
        robot_pose = msg;
    });




    ensta_image = new ROS2D.NavigationImage({
        size: 10.0,
        image: './img/ensta.png' 
    });


    ensta_path = new ROS2D.TraceShape({
        maxPoses: 0
    });

    viewer.addObject(ensta_arrow);

    ensta_pose_param = new ROSLIB.Param({
        ros: ros,
        name: '/mmap/poi/submap_0/ensta'
    });
  

    ensta_pose_param.get(function(value){
        ensta_position = convert_POIPosition_MapPosition(value);
        ensta_arrow.x = ensta_position.pose.position.x;
        ensta_arrow.y = -ensta_position.pose.position.y;
        ensta_arrow.visible = true;
        enstaText.x = ensta_position.pose.position.x;
        enstaText.y = -ensta_position.pose.position.y - 5;
        enstaText.visible = true;
        //viewer.addObject(enstaText);
        ensta_image.x = ensta_position.pose.position.x;
        ensta_image.y = -ensta_position.pose.position.y - 5;
        ensta_image.visible = true;
        viewer.addObject(ensta_image);
        var g = new createjs.Graphics();
        g.setStrokeStyle(1);
        g.beginStroke(createjs.Graphics.getRGB(0,0,0));
        g.beginFill(createjs.Graphics.getRGB(255,0,0));
        g.drawCircle(0,0,3);
    
        var s = new createjs.Shape(g);
        s.x = ensta_position.pose.position.x;
        s.y = -ensta_position.pose.position.y;
    
        viewer.addObject(s);
        var positionVec3 = new ROSLIB.Vector3(ensta_position.pose.position.x, ensta_position.pose.position.y, 0);
        var orientation = new ROSLIB.Quaternion({x:0, y:0, z:0, w:ensta_position.pose.orientation.w});
        var ensta_pose = new ROSLIB.Pose({
            'position' : positionVec3,
            'orientation' : orientation
        });

        var req = new ROSLIB.ServiceRequest({
            start:  {
                header : {
                  frame_id : '/map'
                },
                pose : robot_pose
              },
            goal: {
                header : {
                  frame_id : '/map'
                },
                pose : ensta_pose
              }
        })
    
        glonalPlannerService.callService(req, function(result) {
            console.log("result from service status: " )
            console.log(result);
            for (var x in result.poses) {
                console.log(x);
                ensta_path.addPose(x.pose);
            }
            viewer.addObject(ensta_path);
        });
    });
    

    
    
    pub_web_goto = new ROSLIB.Topic({
        ros:            ros,
        name:           '/web/go_to',
        messageType:    'pal_web_msgs/WebGoTo'
    });

    

}

function convert_POIPosition_MapPosition(position)
{
    pos = {};
    pos.pose = {};
    pos.pose.position = {};
    pos.pose.orientation = {};
	pos.pose.position.x = position[2];
	pos.pose.position.y = position[3];
	pos.pose.orientation.z = Math.sin(position[4] / 2.0);
    pos.pose.orientation.w = Math.cos(position[4] / 2.0);
    return pos;
}
