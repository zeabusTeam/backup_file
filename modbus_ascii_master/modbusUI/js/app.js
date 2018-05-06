var app = angular.module('zeabusModbusMaster', [])
  .controller('modbusController', function($scope,modbus,$interval) {
    
    $scope.devices = []
    $scope.deviceSelected = undefined;
    $scope.HRvalues = ["","","","","","","","","","","","","","",""]

    $scope.scanDevices = function()
    {
      modbus.MBDeviceScan.call({
        data : {},
        success:function(resp){

          var devices = []

          var len = resp.name.length;

          var key_list = Object.keys(resp).map(function (key) {
              return key;
          });

          for(var i=0;i<len;i++)
          {
            var item = {};
            key_list.forEach(function(key){
              item[key] = resp[key][i]
            })
            devices.push(item)
          }
          $scope.devices = devices
          console.log("devices : ",devices);

          document.getElementById('modbus-instuction').style.display = 'none';
          if($scope.deviceSelected == undefined && devices.length >0)
          {
            $scope.deviceSelected = devices[0];
          }
          
          $scope.loadDevice($scope.deviceSelected);
          

          $scope.$apply()
        }
      })
    }    

    $scope.scanDevices()

    $scope.readAllDiscreteInput = function()
    {
      modbus.MBReadAllDiscreteInput.call({
        data:
        {
          device_address : $scope.deviceSelected.address,
        },

        success:function(resp){
          var len = resp.status.length
          for(var i=0;i<len;i++)
          {
            value = bitStatus(resp.status[i])
            $('#DI-status-'+i).text(value)
          }
        }
      })
    }

    $scope.readAllCoil = function()
    {
      modbus.MBReadAllCoil.call({
        data:
        {
          device_address : $scope.deviceSelected.address,
        },
        success:function(resp){
          var len = resp.status.length
          for(var i=0;i<len;i++)
          {
            value = bitStatus(resp.status[i])
            $('#COIL-status-'+i).text(value)
          }
        }
      })
    }

    $scope.readAllInputRegister = function()
    {
      modbus.MBReadAllInputRegister.call({
        data:
        {
          device_address : $scope.deviceSelected.address,
        },
        success: function(resp){
          var len = resp.status.length
          for(var i=0;i<len;i++)
          {
            value = paddingZero(resp.status[i],6)
            $('#IR-status-'+i).text(value)
          }
        }
      })
    }

    $scope.readAllHoldingRegister = function()
    {
      modbus.MBReadAllHoldingRegister.call({
        data:
        {
          device_address : $scope.deviceSelected.address,
        },
        success:function(resp){
          var len = resp.status.length
          for(var i=0;i<len;i++)
          {
            value = paddingZero(resp.status[i],6)
            $('#HR-status-'+i).text(value)
          }
        }
      })
    }

    $scope.loadDevice = function(device)
    {
      $scope.deviceSelected = device;
      console.log("deviceSelected :",$scope.deviceSelected.name,":",$scope.deviceSelected.address);

      $scope.resetAutoScan()

      $scope.readAllDiscreteInput()
      $scope.readAllCoil()
      $scope.readAllInputRegister()
      $scope.readAllHoldingRegister()

      $scope.HRvalues = ["","","","","","","","","","","","","","",""]

      //$scope.$apply()      
    }

    $scope.DIautoScan = false;
    $scope.COILautoScan = false;
    $scope.IRautoScan = false;
    $scope.HRautoScan = false;

    $scope.resetAutoScan = function()
    {

      $scope.DIautoScan = false;
      $scope.COILautoScan = false;
      $scope.IRautoScan = false;
      $scope.HRautoScan = false;
      //$scope.$apply()
    }

    $scope.resetAutoScan()


    $scope.autoUpdateRates = [100,500,1000,2000,5000]
    $scope.autoUpdateRate = 500;
    $scope.Timer = null;

    $scope.setInterval = function(rate) {
      
      if (angular.isDefined($scope.Timer)) {
        $interval.cancel($scope.Timer);
      }
        
      $scope.Timer = $interval(function(){

        console.log("interval update !!");
        if($scope.DIautoScan)
          $scope.readAllDiscreteInput()
        if($scope.COILautoScan)
          $scope.readAllCoil()
        if($scope.IRautoScan)
          $scope.readAllInputRegister()
        if($scope.HRautoScan)
          $scope.readAllHoldingRegister()

      }, rate);
      

    }

    $scope.setInterval($scope.autoUpdateRate)



    $scope.writeCoil = function(address,value)
    {

      modbus.MBWriteCoil.call({
        data: {
                device_address : $scope.deviceSelected.address,
                register_start_address : address ,
                register_quantity : 1,
                status : [Boolean(value)],
              },
        success:function(resp){
          $scope.readAllCoil()
        }
      })
    }


    $scope.writeHR = function(address,value)
    {
      var testList = new Array(true);

      //console.log(address,value);
      modbus.MBWriteHoldingRegister.call(
      {
        data: {
                device_address : $scope.deviceSelected.address,
                register_start_address : address ,
                register_quantity : 1,
                status : [parseInt(value)],
              },
        
        success:function(resp){
          $scope.readAllHoldingRegister()
          $scope.HRvalues[address] = ""
          //$scope.apply()
        }
      })
    }

    
    // angular.element(document).ready(function () {
    //   $('.HR-input').keypress(function (e) {

    //     ng-model="values[$index]"
    //     console.log(e);
    //     if (e.which == 13) {
    //       console.log("TEST");
    //       console.log($(this).data('id'))
    //       return false;    //<---- Add this line
    //     }
    //   });
      
    // });

    $scope.activeRefresh = function (status)
    {
      if(status)
        return "__ACTIVE"
      return "INACTIVE"
    }
  })
  ;