# Architecture Decision Record - Webgui Based Video and Layout
## Status: Proposal

## Context
The development webgui has only minor script changes from the 2018 Polaris source code and launches when it is moved into the 2018 package.
There are minor layout and load order problems in the webgui.
The webgui recieves cv processed images published by vision.
## Decision
Change the load order of bootsrap scripts in the webgui.
Source raw video for the webgui within the webgui node. This is to support a potential webgui/naviation based manual control mode.
src/sender.cpp added using cv bridge image capture and image transfer to load images.
(open) cv bridge and general cpp dependecies added to the webgui. Sender exicutable now created in webgui.
camera_stream.launch added to /ros to launch webgui with webgui sourced video.

## Consequences
The webgui node now launches a camera video for manual control. This will support navigation and machine testing without vision and open cv elements running. As this stream is independent of the vison and related threads the image sample rate may be increased for a smoother monitor video without significant cv processing overhead. The webgui now has conventional bootstrap load order and camera_stream.launch starts webgui based image sampling.
