var ros;
var scl_reach;

var viewer;

var user_arrow;
var ensta_arrow;
var last_user_arrow = new Date(); // Time of last valid reception.

function init() {
    ros = new ROSLIB.Ros({url: 'ws://' + window.location.hostname + ':9090'});

    init_hbba();
    
    viewer = new ROS2D.Viewer({
        divID: 'nav',
        width: 1024,
        height: 860
    });

    var nav = NAV2D.OccupancyGridClientNav({
        ros:        ros,
        rootObject: viewer.scene,
        viewer:     viewer,
        serverName: '/motv_identification_manager/nav_to_pose'
    });

    user_arrow = new ROS2D.NavigationArrow({
        size: 1.0,
        strokeColor: createjs.Graphics.getRGB(0, 0, 255, 0.5) 
    });
    user_arrow.scaleX = 0.1;
    user_arrow.scaleY = 0.1;
    viewer.addObject(user_arrow);


    ensta_arrow = new ROS2D.NavigationArrow({
        size: 1.0,
        strokeColor: createjs.Graphics.getRGB(0, 0, 255, 0.5) 
    });
    ensta_arrow.scaleX = 0.1;
    ensta_arrow.scaleY = 0.1;
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
    });
    

    var user_pose = new ROSLIB.Topic({
        ros:            ros,
        name:           '/interact_pose',
        messageType:    'geometry_msgs/PoseStamped'
    });
    user_pose.subscribe(function(msg) {
        update_user_pose(msg);
    });

    
    pub_web_goto = new ROSLIB.Topic({
        ros:            ros,
        name:           '/web/go_to',
        messageType:    'pal_web_msgs/WebGoTo'
    });

    // Ticker used to check on things such as user pose status.
    createjs.Ticker.addEventListener("tick", ticker_cb);

}

function convert_POIPosition_MapPosition(position)
{
    pos = {};
    pos.pose = {};
    pos.pose.position = {};
    pos.pose.orientation = {};
	pos.pose.position.x = position[0];
	pos.pose.position.y = position[1];
	pos.pose.orientation.z = Math.sin(position[2] / 2.0);
    pos.pose.orientation.w = Math.cos(position[2] / 2.0);
    return pos;
}



function update_user_pose(msg)
{
    user_arrow.x = msg.pose.position.x;
    user_arrow.y = -msg.pose.position.y;
    // Note: If this callback is called, it means we're in the correct state
    // (PI, FR or FP).
    user_arrow.visible = true;
    last_user_arrow = new Date();
}



function go_reach() {
    var loc_e    = document.getElementById("reach_select")
    var loc_name = loc_e.options[loc_e.selectedIndex].value    

    var req = new ROSLIB.ServiceRequest({
        locations:  [loc_name],
        confidence: [1.0]
    })

    scl_reach.callService(req, function(result) {
        console.log("Person status: " + result.status);
    });
}


function ticker_cb(event)
{
    var now = new Date();
    user_elapsed = (now - last_user_arrow) / 1000; // In sec.

    // NOTE: No need to check the state of IDManager, as messages are not
    // produced outside of the PI, FR, and FP states for this topic.
    if (user_elapsed > 1) {
        user_arrow.visible = false;
    }

}

function reset_gui() {
    pub_web_goto.publish({
        type: 3,
        value: "/static/webapps/client/index.html"
    })
}

function stop_all() {
    pub_stop.publish({})
}
