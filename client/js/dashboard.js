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

    


    var enstaText = new createjs.Text("ENSTA STAND", "20px Arial", "#ff7700");
    enstaText.textBaseline = "alphabetic";
    ensta_arrow = new ROS2D.NavigationArrow({
        size: 1.0,
        strokeColor: createjs.Graphics.getRGB(0, 0, 255, 0.5) 
    });
    ensta_arrow.scaleX = 0.1;
    ensta_arrow.scaleY = 0.1;
    viewer.addObject(ensta_arrow);
    viewer.addObject(enstaText);

    ensta_pose_param = new ROSLIB.Param({
        ros: ros,
        name: '/mmap/poi/submap_0/ensta'
    });
  
    ensta_pose_param.get(function(value){
        ensta_position = convert_POIPosition_MapPosition(value);
        ensta_arrow.x = ensta_position.pose.position.x;
        ensta_arrow.y = -ensta_position.pose.position.y;
        ensta_arrow.visible = true;
        enstaText.x = ensta_position.pose.position.x + 2;
        enstaText.y = -ensta_position.pose.position.y;
        enstaText.visible = true;
        var g = new createjs.Graphics();
        g.setStrokeStyle(1);
        g.beginStroke(createjs.Graphics.getRGB(0,0,0));
        g.beginFill(createjs.Graphics.getRGB(255,0,0));
        g.drawCircle(0,0,3);
    
        var s = new createjs.Shape(g);
        s.x = ensta_position.pose.position.x;
        s.y = -ensta_position.pose.position.y;
    
        viewer.addObject(s);
    });
    

    
    
    pub_web_goto = new ROSLIB.Topic({
        ros:            ros,
        name:           '/web/go_to',
        messageType:    'pal_web_msgs/WebGoTo'
    });

    

}

/*țvar that = this;
  options = options || {};
  var size = options.size || 10;
  var strokeSize = options.strokeSize || 3;
  var strokeColor = options.strokeColor || createjs.Graphics.getRGB(0, 0, 0);
  var fillColor = options.fillColor || createjs.Graphics.getRGB(255, 0, 0);
  var pulse = options.pulse;
  // draw the arrow
  var graphics = new createjs.Graphics();
  // line width
  graphics.setStrokeStyle(strokeSize);
  graphics.moveTo(-size / 2.0, -size / 2.0);
  graphics.beginStroke(strokeColor);
  graphics.beginFill(fillColor);
  graphics.lineTo(size, 0);
  graphics.lineTo(-size / 2.0, size / 2.0);
  graphics.closePath();
  graphics.endFill();
  graphics.endStroke();
  // create the shape
  createjs.Shape.call(this, graphics);ț

  create object
   var text = new createjs.Text("Hello World", "20px Arial", "#ff7700");
text.x = 100;
text.textBaseline = "alphabetic";
*/

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
