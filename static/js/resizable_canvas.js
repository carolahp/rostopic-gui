function onloadCallback(topics_and_nodes) {
    var all_topics = topics_and_nodes.topics;
    var all_nodes = topics_and_nodes.nodes;
	var svgDoc = $("#svg-container").contents();
	addClasses(svgDoc, all_topics, all_nodes);
	addListeners(svgDoc);
    initSvgPanZoom(generatePanZoomEventsHandler());
}

function generatePanZoomEventsHandler() {
    var eventsHandler;
    eventsHandler = {
      haltEventListeners: ['touchstart', 'touchend', 'touchmove', 'touchleave', 'touchcancel']
    , init: function(options) {
        var instance = options.instance
          , initialScale = 1
          , pannedX = 0
          , pannedY = 0
        // Init Hammer
        // Listen only for pointer and touch events
        this.hammer = Hammer(options.svgElement, {
          inputClass: Hammer.SUPPORT_POINTER_EVENTS ? Hammer.PointerEventInput : Hammer.TouchInput
        })
        // Enable pinch
        this.hammer.get('pinch').set({enable: true})
        // Handle double tap
        this.hammer.on('doubletap', function(ev){
          instance.zoomIn()
        })
        // Handle pan
        this.hammer.on('panstart panmove', function(ev){
          // On pan start reset panned variables
          if (ev.type === 'panstart') {
            pannedX = 0
            pannedY = 0
          }
          // Pan only the difference
          instance.panBy({x: ev.deltaX - pannedX, y: ev.deltaY - pannedY})
          pannedX = ev.deltaX
          pannedY = ev.deltaY
        })
        // Handle pinch
        this.hammer.on('pinchstart pinchmove', function(ev){
          // On pinch start remember initial zoom
          if (ev.type === 'pinchstart') {
            initialScale = instance.getZoom()
            instance.zoom(initialScale * ev.scale)
          }
          instance.zoom(initialScale * ev.scale)
        })
        // Prevent moving the page on some devices when panning over SVG
        options.svgElement.addEventListener('touchmove', function(e){ e.preventDefault(); });
      }
    , destroy: function(){
        this.hammer.destroy()
      }
    }
    return eventsHandler;
}

function initSvgPanZoom(eventsHandler) {
    svgPanZoom('#svg-container', {
              zoomEnabled: true,
              controlIconsEnabled: true,
              preventMouseEventsDefault: false,
              dblClickZoomEnabled: false,
              mouseWheelZoomEnabled: false,
              fit: 1,
              center: 1,
              customEventsHandler: eventsHandler
            });      
}

function getROSElementName(title, all_elements) {
    if( $.type(title) !== "string" || title === "" || title === null) {
        return false;
    }
    possibleNames = title.split('-&gt;');
    for(var i = 0; i < possibleNames.length; i++) {
        if($.inArray(possibleNames[i], all_elements) > -1) {
        return possibleNames[i].replace(/\//g,"-");
        }
    }
    //console.error("Element associated with no topic" + possibleNames);
    return false;
}

function addClasses(svgDoc, all_topics, all_nodes) {
    var allElemsG = svgDoc.find("g");

	/* iterate G elements */
	for(var i = 0; i < allElemsG.length; i++)	{
	    	
		
	    var elm = allElemsG[i];
	    var elemG = $(elm);
	    
	    var title = elemG.find("title").html();
		if(! title) {
		    continue;
		}
		var className = "";
		/* graph */
		if(elemG.attr("class") == "graph") {
            continue;
		}
		/* edge */
		if(elemG.attr("class") == "edge") {
    		className = "topic" + getROSElementName(title, all_topics);
		}
		/* node */
		if(elemG.attr("class") == "node") {
		    var isTopicName = getROSElementName(title, all_topics);
		    if(! isTopicName ) {
		        var isNodeName = getROSElementName(title, all_nodes);
		        if(! isNodeName ) {
		            //console.error(title + " isn't topic nor node");
		            continue;
		        }
		        else {
		            className = "node" + isNodeName;
		        }
		    }
		    else {
		        className = "topic" + isTopicName;
		    }
		}

	    var gChildren = [];
        var polygons = elemG.find("polygon");	
        //console.log("length polygons " +  polygons.length);
        if(polygons.length > 0) {
            gChildren.push(polygons[0]);
        }
		var paths = elemG.find("path");	
        if(paths.length > 0) {
            gChildren.push(paths[0]);
        }
		var texts = elemG.find("text");	
        if(texts.length > 0) {
            gChildren.push(texts[0]);
        }
		
		/* iterate polygons and paths found in G */
		for(var j = 0; j < gChildren.length; j++) {
		    var child = $(gChildren[j]);
		    child.attr("class", className);		    
		}
	}
}

function addListeners(svgDoc) {
    var allElemsG = svgDoc.find("g");

	/* iterate G elements */
	for(var i = 0; i < allElemsG.length; i++) {
	    
	    var elm = allElemsG[i];
	    var elemG = $(elm);
	    /* graph */
		if(elemG.attr("class") == "graph") {
            continue;
		}
	    var gChildren = [];
        var polygons = elemG.find("polygon");	
        //console.log("length polygons " +  polygons.length);
        if(polygons.length > 0) {
            gChildren.push(polygons[0]);
        }
		var paths = elemG.find("path");	
        if(paths.length > 0) {
            gChildren.push(paths[0]);
        }
		var texts = elemG.find("text");	
        if(texts.length > 0) {
            gChildren.push(texts[0]);
        }
		
		/* iterate polygons and paths found in G */
		for(var j = 0; j < gChildren.length; j++) {
		    var child = $(gChildren[j]);
		    //console.log(child);
		    //console.log(child.attr("class"));
		    var addListener = function(child_param, class_name_param) {
		        var class_name_copy = class_name_param;
		        if(! class_name_copy) {
    		        //to exclude rosout_agg
    		        return;
    		    }
    		    
		        var jclass = '.' + class_name_param;
	            //console.log("jclass final " + jclass);
	            if(jclass == ".") {
	                console.error(this);
	            }
                var classElements = svgDoc.find(jclass);
                
		        child_param.mouseover(function() {
		        	classElements.each(function(i, obj) {
                        generateStrokeFun(obj, "on")();
                    });
    		    });
    		    
    		    child_param.mouseout(function() {
		        	classElements.each(function(i, obj) {
                        generateStrokeFun(obj, "off")();
                    });
    		    });
    		    var topicName = getTopicFromClass(class_name_copy);
    		    var nodeName = getNodeFromClass(class_name_copy);
    		    if(topicName) {
    		        child_param.click(function() {
		            	generateClickFun("topic", topicName)();
    		        });
    		    }
    		    else if(nodeName){
    		        child_param.click(function() {
		            	generateClickFun("node", nodeName)();
    		        });
    		    }
    		    else {
    		        console.error("element isnt topic nor node, and click handler wasn't added");
    		    }
		    }
		    addListener(child, child.attr("class"));
		}
	}
}

function getTopicFromClass(class_name) {
    if(class_name.indexOf("topic-") < 0) {
        return false;
    }
    return "/" + class_name.substring(6, class_name.length).replace(/-/g,"/");
}
function getNodeFromClass(class_name) {
    if(class_name.indexOf("node-") < 0) {
        return false;
    }
    return "/" + class_name.substring(5, class_name.length).replace(/-/g,"/");
}
function generateClickFun(type, ros_name) {
    return function() {
        if(type === "node") {
            $.selected_topic = null;
			$('#topic_msg_type').html('Node: ' + ros_name +'<br>&nbsp;');
			$("#echo_button").prop('disabled', true);
        }
        else if(type === "topic") {
            $.getJSON('/get_msg_type', {
			      topic: [ros_name]
			    }, function(data) {
				    $.selected_topic = data.real_topic;
			      $('#topic_msg_type').html('Topic: ' + data.real_topic + '<br>Type: ' + data.topic_type);
			      $("#echo_button").prop('disabled', false);});
        }
        else {
            $("#echo_button").prop('disabled', true);
            console.error("click handler added nor a topic neither a node");
        }
        
    };
    
}

function generateStrokeFun(elem, new_state) {
	return function() {
	    var parentClass = elem.parentElement.getAttribute("class");
	    var elemClass = elem.getAttribute("class");
	    var isNode = elemClass.indexOf("node-") > -1;
	    var color = "";
	    if(new_state == "on") {
	        color = (isNode ? "blue" : (parentClass == "edge" ? "crimson" : "red"));
	    }
	    else if(new_state == "off") {
	        color = "black";
	    }
	    else {
	        console.error("new_state is not defined");
	        return;
	    }

	    var elemTag = elem.tagName;
		switch (elemTag) {
		    case "polygon":
	            if(parentClass == "edge") {
        		    elem.setAttribute("fill", color);		            
	            }
		        elem.setAttribute("stroke", color);
		        break;
		    
		    case "text":
		        elem.setAttribute("fill", color);		            
		        break;
		    
		    case "path":
		        elem.setAttribute("stroke", color);
		        break;
		}
    }
}


