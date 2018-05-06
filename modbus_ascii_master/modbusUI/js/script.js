

var ros = new ROSLIB.Ros();



// If there is an error on the backend, an 'error' emit will be emitted.
ros.on('error', function(error) {
  document.getElementById('connecting').style.display = 'none';
  document.getElementById('connected').style.display = 'none';
  document.getElementById('closed').style.display = 'none';
  document.getElementById('error').style.display = 'inline';
  console.log(error);
});
// Find out exactly when we made a connection.
ros.on('connection', function() {
  console.log('Connection made!');
  document.getElementById('connecting').style.display = 'none';
  document.getElementById('error').style.display = 'none';
  document.getElementById('closed').style.display = 'none';
  document.getElementById('connected').style.display = 'inline';
  
  document.getElementById('modbus-instuction').style.display = 'block';
  document.getElementById('instuction').style.display = 'none';
  
  
});
ros.on('close', function() {
  console.log('Connection closed.');
  document.getElementById('connecting').style.display = 'none';
  document.getElementById('connected').style.display = 'none';
  document.getElementById('closed').style.display = 'inline';
});


// Create a connection to the rosbridge WebSocket server.
ros.connect('ws://localhost:9090');


$( document ).ready(function() {
  $("#hostname").focus()
  $("#hostname").change(function(){
    hostname=$(this).val()
    ros.close()
    ros.connect('ws://'+hostname+':9090');
  });
});


zeabus_modbus_master = 
[
  { 
    name : 'zeabus_modbus_master/MBDeviceScan',
     srv : 'modbus_ascii_ros/DeviceDiscovered'
  },
  { 
    name : 'zeabus_modbus_master/MBReadAllCoil',
     srv : 'modbus_ascii_ros/MBBitRegisterStatus'},
  { 
    name : 'zeabus_modbus_master/MBReadAllDiscreteInput',
     srv : 'modbus_ascii_ros/MBBitRegisterStatus'},
  { 
    name : 'zeabus_modbus_master/MBReadAllHoldingRegister',
     srv : 'modbus_ascii_ros/MBWordRegisterStatus'},
  { 
    name : 'zeabus_modbus_master/MBReadAllInputRegister',
     srv : 'modbus_ascii_ros/MBWordRegisterStatus'},
  { 
    name : 'zeabus_modbus_master/MBWriteCoil',
     srv : 'modbus_ascii_ros/MBBitRegisterCommand'},
  { 
    name : 'zeabus_modbus_master/MBWriteHoldingRegister',
     srv : 'modbus_ascii_ros/MBWordRegisterCommand'
  }
]

$(document).ready(function() {
    $('select').material_select();
});

function paddingZero(num, size) {
    var s = num+"";
    while (s.length < size) s = "0" + s;
    return s;
}


function bitStatus(status)
{
  if(status)
    return "SET"
  return "RESET"
}