# ROS2-Django-Bridge

```bash
 /======================================================\ 
||           ____     ____     _____   ______           || 
||          |  _ \   / __ \   /   __| |___   \          || 
||          | |_| | | |  | |  |  |        |  |          || 
||          |    /  | |  | |   \  \      /  /           || 
||          | |\ \  | |__| |  __|  |    /  /__          || 
||          |_| \_\  \____/  |_____/   |______|         || 
 \====================             =====================/ 
      ~          ~~  \\            \\  ~     ~~~         
            ~~~~      \\            \\      ~~           
     ~~                ||            ||          ~~      
         ~~~           ||            ||   ~~~~           
               ~~      ||            ||       ~~         
                   ~~ //            //   ~~       ~      
      ~          ~~~~//            // ~~       ~~~~      
 /====================             =====================\ 
||  ____      ____     _      __   __   _____    ____   ||
|| |  _ \     |  |    / \    |  \ |  | / ____|  / __ \  ||
|| | | | |    |  |   / _ \   |   \|  || |      | |  | | ||
|| | | | | _  |  |  / /_\ \  |    \  || |   __ | |  | | ||
|| | |_| || |_|  | /  ___  \ |  |\   || |__|_ || |__| | ||
|| |____/ |______|/__/   \__\|__| \__| \_____/  \____/  ||
 \======================================================/ 
```

This is a Django-based web interface for a ROS2 system, providing both REST API endpoints and web pages. It allows for a user-friendly way to interact with a ROS2 system, and is written in Python.

## Development/Contributions

This project is in the early stages of development. Any Contributions are welcome.

## Django-Overview

Django is a great choice for building a web interface for a ROS2 system because it provides a high-level framework for building web applications quickly and efficiently. It includes many features such as an ORM, templates, and a robust URL routing system, which can help streamline development and make it easier to maintain the codebase. Additionally, Django has a large and active community, which can provide valuable resources and support for developers.

- The django portion of this package will contain two apps:
    1. bridge_api
        - This app will provide various REST API endpoints for interacting with the ROS2 system.
    2. bridge_interface
        - This app will provide similar functionality, but in the form of rendered web pages that can be used in a mobile app or viewed from a web browser on the same network.

## ROS2-Overview

The ROS2 system is used to manage the communication between the web interface and the actual ROS2 system. It provides a set of tools and libraries for building and managing ROS2 systems, and it includes a message passing protocol for communication between nodes.

- This package is built using ROS2 Humble and python.

## Getting Started

- Ensure you have django, django-rest, and ros2-humble dependencies installed.
- Run the launch script

```bash
source launch.sh
```

## Notes

- This package is not Thread Safe.

## Leo, summarize this repo in the style of Dr. Seuss

```text
In a land of tech and code,
Where ROS2 and Django roam,
A web interface is born,
To make life easier, no need to mourn.

Django, a framework so fine,
Helps build web apps in no time,
With ORM and templates, it's a breeze,
And a community that's always on the ease.

ROS2, a system so grand,
Manages communication, hand in hand,
Message passing, it's all so neat,
Making life easier, can't be beat.

So if you're looking for a way,
To make your ROS2 system sway,
Just use this web interface, it's the best,
And you'll be on your way, no need to stress!

- Leo by Brave
```
