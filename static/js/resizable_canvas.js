function onloadCallback() {
	addListeners();
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
function addListeners(){
    var a = document.getElementById("svg-container");
    var svgDoc = a.contentDocument;
	var allElemsG = svgDoc.getElementsByTagName("g");
	/* iterate G elements */
	for(var i = 0; i < allElemsG.length; i++)	{
		var elemG = allElemsG[i];
		if(elemG.getAttribute("class") == "edge") {
			var eGChildren = elemG.childNodes;
			var edgeElems = [];
			var topicName = "";

			for(var j = 0; j < eGChildren.length; j++) {
				var child = eGChildren[j];
				/* get topic name */
				if(child.tagName == "title") {
					topicName = child.innerHTML.split('-&gt;');
				}
				/* save polygons and paths found inside G (arrow parts) */
				if(child.tagName == "polygon" || child.tagName == "path") {
					edgeElems.push(child);
				}
			}
			/* iterate polygons and paths found in G */
			for(var j = 0; j < edgeElems.length; j++) {
				var colorStroke = function() {
					var childCopy = edgeElems[j];
					var edgeElemsCopy = edgeElems;
					var topicNameCopy = topicName;
					/* function that changes the arrow color */
					var colorStrokeFun = generateStrokeFun(edgeElemsCopy)
					/* add events listeners */
					addListenersToElement(childCopy, topicNameCopy, colorStrokeFun)
				}
				colorStroke();
			}
		}
	}
}

function addListenersToElement(element, topicName, colorStrokeFun) {
	element.addEventListener('click', function(event) { $.getJSON('/get_msg_type', {
													  topic: topicName
													}, function(data) {
														$.selected_topic = data.real_topic;
													  $('#topic_msg_type').html('Topic: ' + data.real_topic + '<br>Type: ' + data.topic_type);
													});});
	element.addEventListener('mouseover', function (event) { colorStrokeFun("red")});
	element.addEventListener('mouseout', function (event) { colorStrokeFun("black")});
}

function generateStrokeFun(edgeElems) {
	return function(color) {
			for(var k = 0; k < edgeElems.length; k++){
				var childCp = edgeElems[k];
				childCp.setAttribute("stroke", color);
				if(childCp.tagName == "polygon")
					childCp.setAttribute("fill", color);
			}
	}
}


