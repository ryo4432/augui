const initTime = Date.now();
const historyCount = 100;

let staticParams = {
  region: {
    originLatitude: 0.0,
    originLongitude: 0.0,
    height: 0.0,
    width: 0.0,
    direction: 0.0
  },
  toRectanglePosition: function() {
    let xys = []
    console.log(this.region.direction / 180.0 * Math.PI)
    xys.push(transform2d([this.region.height, this.region.width], this.region.direction / 180.0 * Math.PI))
    xys.push(transform2d([-this.region.height, this.region.width], this.region.direction / 180.0 * Math.PI))
    xys.push(transform2d([-this.region.height, -this.region.width], this.region.direction / 180.0 * Math.PI))
    xys.push(transform2d([this.region.height, -this.region.width], this.region.direction / 180.0 * Math.PI))
    lls = [];
    xys.forEach(xy => {
      lls.push(xy2ll([this.region.originLatitude, this.region.originLongitude], xy))
    });
    return lls
  }
}

let topics = {
  time: 0.0,
  latitude: 35.056796,
  longitude: 138.765736,
  altitude: 0.0,
  position : {
    x: 0.0,
    y: 0.0,
    z: 0.0
  },
  orientation: {
    euler: {
    roll: 0.0,
    pitch: 0.0,
    yaw: 0.0
    },
    quat: {
      x: 0.0,
      y: 0.0,
      z: 0.0,
      w: 1.0,
    }
  },
  velocity : {
    x: 0.0,
    y: 0.0,
    z: 0.0
  },
  angular_velocity: {
    roll: 0.0,
    pitch: 0.0,
    yaw: 0.0
  },
  quat2rpy: function() {
    let q = new Quaternion(this.orientation.quat);
    this.orientation.euler = q.toEuler();
  },
  power: {
    thruster_mt_drive: false,
    thurster_mt_contro: false,
    left_upper: false,
    left_lower: false,
    right_upper: false,
    right_lower: false,
    ins: false,
    dvl: false,
    gps: false,
    sonar: false
  }
};
// auv position on map
const auv_img = L.icon({ iconUrl: 'js/img/auv.drawio.svg', iconRetinaUrl: 'js/img/auv.drawio.svg', iconSize: [30, 30], iconAnchor: [25, 25], popupAnchor: [0, -50] });
let auv_pos = {};

const rosParams = {
  mockup: [
    'origin_lat',
    'origin_lon',
    'region_height',
    'region_width',
    'region_direction'
  ]
};

// onoff power name
const powerNames = [
  "thruster",
  "left_upper",
  "left_lower",
  "right_upper",
  "right_lower",
  "ins",
  "dvl",
  "gps",
  "fls"
]

// websocket
let ws = new WebSocket("ws://localhost:9090");
// map object
let polygon;
let route;
let circle;
let map = L.map('content').setView([topics.latitude, topics.longitude], 13);

// ws initialize
ws.onopen = function() {
  // publisher
  ws.send(JSON.stringify({"op": "advertise", "topic": "/cmd_vel", "type": "geometry_msgs/msg/Twist"}));
  // subscriver
  ws.send(JSON.stringify({"op": "subscribe", "topic": "/gps"}));
  ws.send(JSON.stringify({"op": "subscribe", "topic": "/odom"}));
  ws.send(JSON.stringify({"op": "subscribe", "topic": "/altitude"}));
  for (const pn of powerNames) {
    ws.send(JSON.stringify({"op": "subscribe", "topic": "/power/" + pn}));
  }
  // service client
  ws.send(JSON.stringify({
    "op": "call_service",
    "service": "/mockup/get_parameters",
    "args": {
      "names": rosParams.mockup
  }}))
  ws.send(JSON.stringify({"op": "call_service", "service": "/get_waypoints"}))
}
ws.onmessage = function(e) {
  const data = JSON.parse(e.data);
  if (data.op == "publish") {
    if (data.topic == "/gps") {
    topics.latitude = data.msg.latitude;
    topics.longitude = data.msg.longitude;
    if (auv_pos != undefined && map != undefined) {
      map.removeLayer(auv_pos);
    }
    auv_pos = L.marker(
      [topics.latitude, topics.longitude],
      {
        icon: auv_img,
        rotationAngle: topics.orientation.euler.yaw / Math.PI * 180.0 - 90.0
      }).addTo(map);
    return;
    }
    if (data.topic == "/odom") {
      topics.position = data.msg.pose.pose.position;
      topics.orientation.quat = data.msg.pose.pose.orientation;
      topics.quat2rpy();
      topics.velocity = data.msg.twist.twist.linear;
      topics.angular_velocity = data.msg.twist.twist.angular;
      return;
    }
    if (data.topic == "/altitude") {
      topics.altitude = data.msg.data;
      return;
    }
    for (pn of powerNames) {
      if (data.topic.lastIndexOf(pn) > -1) {
        const arr = data.topic.split(["/"]);
        topics.power[arr[arr.length - 1]] = data.msg.data;
        const $btn = document.getElementById("onoff-" + arr[arr.length - 1]);
        if (data.msg.data) {
          $btn.style.backgroundColor = '#04AA6D';
        } else {
          $btn.style.backgroundColor = '#f4511e';
        }
        return;
      }
    }
  }
  if (data.op == "service_response") {
    if (data.service == "/mockup/get_parameters") {
      staticParams.region.originLatitude = data.values.values[0].double_value
      staticParams.region.originLongitude = data.values.values[1].double_value
      staticParams.region.height = data.values.values[2].double_value
      staticParams.region.width = data.values.values[3].double_value
      staticParams.region.direction = data.values.values[4].double_value
      polygon = L.polygon(staticParams.toRectanglePosition(), {fillColor: "none"}).addTo(map);
      // circle = L.circle(
      //   [staticParams.region.originLatitude + 0.02, staticParams.region.originLongitude + 0.02], {
      //   color: 'green',
      //   weight: 0.5,
      //   fillColor: 'green',
      //   fillOpacity: 0.5,
      //   radius: 500
      // }).addTo(map);
      return;
    }
    if (data.service == "/get_waypoints") {
      console.log(data)
      ll = []
      data.values.waypoints.forEach(el => {
        ll.push([el.latitude, el.longitude])
      });
      route = L.polyline(ll,{
        "color": "#FF0000",
        "weight": 2,
        "opacity": 0.3,
      }).addTo(map);
    }
  }
}

// map initialize
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '© OpenStreetMap'
}).addTo(map);

/*
  chart
*/
const ctx = document.getElementById('depthChart');
let depthChart = new Chart(ctx, {
    type: 'line',
    data: {
        labels: [],
        datasets: [{
          label: 'depth[m]',
          data: [],
          fill: false,
          borderColor: 'rgb(255, 99, 132)',
          tension: 0.1,
        },
        {
          label: 'altitude[m]',
          data: [],
          fill: false,
          borderColor: 'rgb(54, 162, 235)',
          tension: 0.1,
      }]
    },
    options: {
      responsive: true,
      aspectRatio: 8,
        legend: {
          display: true,
          position: 'chartArea',
          align: 'start',
          labels: {
              fontColor: '#333'
          }
      },
      tooltip: {
        
      },
      scales: {
        xAxes: [{
          scaleLabel: {
            display: true,
            labelString: 'Time[s]'
          }
        }]
      }
    }
});

function updateData(chart, label, data, size=historyCount) {
  chart.data.labels.push(label);
  let index = 0;
  chart.data.datasets.forEach((dataset) => {
      dataset.data.push(data[index]);
      index++;
  });
  if (chart.data.labels.length > size) {
    chart.data.labels.shift();
    chart.data.datasets.forEach((dataset) => {
        dataset.data.shift();
    });
  }
  chart.update();
}

setInterval(function() {
  let elapsedTimeSec = Math.floor((Date.now() - initTime) / 1000);
  updateData(depthChart, elapsedTimeSec.toString(), [topics.position.z, topics.altitude]);
}, 1000);

function onOffSwitch(id) {
  const arr = id.split("-");
  const name = arr[arr.length - 1];
  const srv = {
    "op": "call_service",
    "service": "/set_power_" + name,
    "args": {
      "data": !topics.power[name]
    }
  };
  ws.send(JSON.stringify(srv));
}

let joyStatus = 0;
let joyConnected = false;

$(document).ready(function() {
  if("getGamepads" in navigator) {
    const $btn = document.getElementById("joy");
    $(window).on("gamepadconnected", function() {
      joyConnected = true;
      $btn.style.backgroundColor = '#04AA6D';
    });

    $(window).on("gamepaddisconnected", function() {
      joyConnected = false;
      $btn.innerText = "joystick\nno control";
      $btn.style.backgroundColor = '#f4511e';
    });
  }
});

function joySwitch() {
  if (joyConnected) {
    joyStatus = (joyStatus + 1) % 3;
  }
  else {
    joyStatus = 0;
  }

  const $btn = document.getElementById("joy");
  if (joyStatus == 0) {
    $btn.innerText = "joystick\nno control";
  }
  if (joyStatus == 1) {
    $btn.innerText = "joystick\ntwist";
  }
  if (joyStatus == 2) {
    $btn.innerText = "joystick\nactuator";
  }
}

setInterval(function() {
  if (!joyConnected) {
    return;
  }
  let gp = navigator.getGamepads()[0];
  if (joyStatus == 1) { // publish cmd_vel value
    // left stick:
    // idx=0 -> right:1, left:-1
    // idx=1 -> up:-1, down:1, 
    // right stick:
    // idx=3 -> right:1, left:-1
    // idx=4 -> up:-1, down:1, 
    const maxVel = 1.5;
    const maxRate = 10.0;
    velX = -1 * gp.axes[1] * maxVel;
    yawRate = gp.axes[2] * maxRate;
    pitchRate = -1 * gp.axes[3] * maxRate;
    cmd_vel = {
      linear: {
        x: velX,
        y: 0.0,
        z: 0.0
      },
      angular:
      {
        x: 0.0,
        y: pitchRate,
        z: yawRate
      }
    }
    ws.send(JSON.stringify({ "op": "publish", "topic": "/cmd_vel", "msg": cmd_vel}));
  }
  if (joyStatus == 2) { // publish actuator value

  }
}, 200);

// marker.bindPopup("<b>Hello world!</b><br>I am a popup.").openPopup();
// circle.bindPopup("I am a circle.");
// polygon.bindPopup("I am a polygon.");
// let popup = L.popup()
//     .setLatLng([51.513, -0.09])
//     .setContent("I am a standalone popup.")
//     .openOn(map);
// function onMapClick(e) {
//   alert("You clicked the map at " + e.latlng);
// }