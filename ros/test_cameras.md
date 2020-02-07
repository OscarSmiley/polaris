# Architecture Decision Record - Webgui and Camera Tests
## Status: Proposal

## Context
The development webgui has only minor script changes from the 2018 Polaris source code and launches when it is moved into the 2018 package.
There are minor layout and load order problems in the webgui. The camera_test.launch requres sender and reciever exicutables to fuction in the development branch.
## Decision
change the load order of bootsrap scripts in the webgui. Bring over sender and reciever files from the 2018 code base and configure the their exicutables in the vision node.
## Consequences
As a result of the changes the webgui now conforms to convention on bootstrap load order and the camera_test.launch file starts in the development branch.
However the sender and reciever additions may only be used by camera testing files. 
