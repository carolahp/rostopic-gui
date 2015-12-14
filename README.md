# WebTopic
## Introduction
WebTopic is an [rqt_graph] (http://wiki.ros.org/rqt_graph) like interface, but in your browser.

It's intended to help developers see and interact with messages going through the ROS nodes. Usually robots are on the run, so you can follow them while watching the messages on your phone. WebTopic was designed with mobile devices in mind, and yet it will also work on your laptop or desktop computer.

## A little bit more about WebTopic
### How does it work?
It's pretty similar to rqt_graph and the topic_monitor plugin of rqt. WebTopic uses the same core functions as these plugins, with only slight variations in some cases. The main difference is the interface--we provide an HTML interface suitable for mobile devices and computers instead of python qt.

[Flask] (http://flask.pocoo.org/) is being used as our web framework and [Werkzeug] (http://werkzeug.pocoo.org/) as our WebServer. 

### Why did you write it?
This software was initially developed as the main project in CC-5407 - "Software Engineering for Robotics," a semestral course at the Faculty of Physical and Mathematical Sciences, Universidad de Chile. Its main purpose was trying to make the debug process of a moving robot easier.

## Features
* Graph with topic interactions.
* Topic listening of the messages.
* JSON-HTTP API to get topics information and/or listen to them.

## Install
- Clone the repository: `git clone https://github.com/carolahp/rostopic-gui.git`
- Go to the directory where it was cloned.
- Install the dependencies: `pip install -r requirements.txt`
- Run it! `python webtopic.py`
- Go to http://localhost:5000 on your favourite browser and check it out.
- It will also be available for all the devices in your local network, just go to http://IP_PC_RUNNING_WEBTOPIC:5000 . 

## Contribute
1. Fork it!
2. Create your feature branch: `git checkout -b my-new-feature`
3. Commit your changes: `git commit -a -m 'Add my feature'`
4. Push to the branch: `git push origin my-new-feature`
5. Send us a pull request

## Original authors
* Carolina Hernández <cahernan@dcc.uchile.cl>
* Francisco Montoto <fmontotomonroy@gmail.com>

## License
This software is distributed under the BSD license, see LICENSE.
