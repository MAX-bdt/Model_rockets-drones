/*
Functions related to JACK connector (flight detector)
*/

//function to verify jack connection
void jackConnected(){
  bool jackUnplugged =0;
  while (jackUnplugged !=1){
    if (digitalRead(jack)==1){//jack plugged
      jackUnplugged=1;
      }
  }
}
