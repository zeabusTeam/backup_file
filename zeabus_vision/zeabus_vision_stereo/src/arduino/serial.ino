byte lastIndex = 0xFF;  // init value, 0xF0 .. 0xFF
boolean checkIndex(byte index){
  if (index == lastIndex+1){
    // in sync
    lastIndex = index;  // update current index
    return true;
  } else {
    // not correct or overflow
    if (lastIndex == 0xFF && index == 0xF0){
      // overflow
      lastIndex = 0xF0;
      return true;
    }
    // not correct
    if (index <= 0xFF && index >= 0xF0){
      //in range, out of sync
      lastIndex = index;
    } else {
      lastIndex = 0xFF;
    }
    return false;
  }
}

