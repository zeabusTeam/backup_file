app.factory('modbus', function() {
     

	var modbus = {}

	zeabus_modbus_master.forEach(function(service){

	  var name = service.name.split("/").slice(-1)[0]

	  modbus[name] = {
	    client : new ROSLIB.Service({
	      ros : ros,
	      name : service.name,
	      serviceType : service.srv
	    }),
	    call : function(options)
	    {	      

	      if(!options.success) options.success = function(){};
	      if(!options.error) options.error = function(){};
	
	      options.debug = true;
	      if(options.debug)
	      {
	      	console.log(service.name)
	        console.log("request",options.data)
	      }
	      this.client.callService(new ROSLIB.ServiceRequest(options.data), 
	      function(resp){

	      	if(options.debug)
	      	console.log("respone",resp);

	      	options.success(resp);

	      	//return resp;
	      },
	      function(resp)
	      {	
	      	if(options.debug)
	      	console.log("error",resp);
	      	//options.error(resp);
	      	$("<tt></tt>").text(resp).prependTo('#log').delay(10000)
	      	.queue(function() { $(this).remove(); });



	      })
	    }
	  }

	})

	return modbus;
});

app.filter('range', function() {
  return function(input, total) {
    total = parseInt(total);

    for (var i=0; i<total; i++) {
      input.push(i);
    }

    return input;
  };
});
