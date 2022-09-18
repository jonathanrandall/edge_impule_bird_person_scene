# edge_impule_bird_person_scene
classifies bird or person or scene in an image

If compiling in Arduino IDE, make sure you are using esp32 board version 1.0.6. 
Don't use version 2.

Also, for TFT_eSPI library, make sure to change your User_setup.h file. I've included mine here.

My EdgeImpulse library bird_classification_3_inferencing, should be placed in the Arduino/libraries folder. 
If you are going to include your own inferencing model, you need to edit the *ei_classifier_porting.cpp* file in:
C:\Users\\_yourname_\Documents\Arduino\libraries\bird_classification_3_inferencing\src\edge-impulse-sdk\porting\arduino
You need to change the ei_calloc function to look like:

 > \__attribute__((weak)) void *ei_calloc(size_t nitems, size_t size) {  
 >      if(psramFound()){
 >          return ps_calloc(nitems, size);   
 >     } 
 >
 >          return calloc(nitems, size);   
 > } 
