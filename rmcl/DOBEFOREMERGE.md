# TODOs before merge noetic-dev -> noetic

- [X] keep simple findRCC functions (fewer arguments)! or implement alternative structure that includes a dynamic number of arguments. If the interface changes because of that, we need to increase a minor version number
- [X] integrate rmagine svd/umeyama functions
- [ ] implement / copy-paste all new implementations of object-based RCC correspondences to other embree corrector classes
- [X] there is always "scene_id". In rmagine, however, it is called object_id. Check this equalize this -> we agreed on using the first level in the scene graph to define the object ids. As it is done in rmagine already.
- [ ] carefully recap if changes effect certain applications
- [ ] delete this file
