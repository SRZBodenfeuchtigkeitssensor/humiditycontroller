function Decode(fPort, bytes, variables) {
  var retValue = {};
  
  if (fPort == 5) {
    retValue.batt = bytes[0] / 10.0; 
    if (retValue.batt === 0||retValue.batt>1000) delete retValue.batt; 

    if (bytes.length >= 3) {
      retValue.humidity = ((bytes[1] << 8) | bytes[2]) / 10;
      if (retValue.humidity === 0||retValue.humidity>1000) delete retValue.humidity; 
    }
      
    if (bytes.length >= 4) {
      retValue.temperature = (((bytes[3] << 8) | bytes[4]) - 400) / 10;
      if (retValue.temperature === 0||retValue.temperature > 1000) delete retValue.temperature; 
    }
      
    if (bytes.length >= 6) { 
      retValue.pressure = ((bytes[5] << 8) | bytes[6]) / 10; 
      if (retValue.pressure === 0) delete retValue.pressure; 
    }
      
    if (bytes.length >= 7) { 
      retValue.minWert1 = bytes[7]; 
      if (retValue.minWert1 <= 0||retValue.minWert1>255) delete retValue.minWert1; 
    }
    
    if(bytes.length >= 8) {
      retValue.maxWert1 = bytes[8]+75;
      if (retValue.maxWert1 < 75||retValue.maxWert1>330) delete retValue.maxWer1; 
    }

    if(bytes.length >= 9) {
      retValue.frequenz1 = bytes[9];
      if (retValue.frequenz1 <= 0||retValue.frequenz1>255) delete retValue.frequenz1; 
    }

    if (bytes.length >= 10) { 
      retValue.minWert2 = bytes[10]; 
      if (retValue.minWert2 <= 0||retValue.minWert2>255) delete retValue.minWert2; 
    }
    
    if(bytes.length >= 11) {
      retValue.maxWert2 = bytes[11]+75;
      if (retValue.maxWert2 < 75||retValue.maxWert2>330) delete retValue.maxWert2; 
    }

    if(bytes.length >= 12) {
      retValue.frequenz2 = bytes[12];
      if (retValue.frequenz2 <= 0||retValue.frequenz2>255) delete retValue.frequenz2; 
    }

    if (bytes.length >= 13) { 
      retValue.minWert3 = bytes[13]; 
      if (retValue.minWert3 <= 0||retValue.minWert3>255) delete retValue.minWert3; 
    }
    
    if(bytes.length >= 14) {
      retValue.maxWert3 = bytes[14]+75;
      if (retValue.maxWert3 < 75||retValue.maxWert3>330) delete retValue.maxWert3; 
    }

    if(bytes.length >= 15) {
      retValue.frequenz3 = bytes[15];
      if (retValue.frequenz3 <= 0||retValue.frequenz3>255) delete retValue.frequenz3; 
    }

    if (bytes.length >= 16) { 
      retValue.minWert4 = bytes[16]; 
      if (retValue.minWert4 <= 0||retValue.minWert4>255) delete retValue.minWert4; 
    }
    
    if(bytes.length >= 17) {
      retValue.maxWert4 = bytes[17]+75;
      if (retValue.maxWert4 < 75||retValue.maxWert4>330) delete retValue.maxWert4; 
    }

    if(bytes.length >= 18) {
      retValue.frequenz4 = bytes[18];
      if (retValue.frequenz4 <= 0||retValue.frequenz4>255) delete retValue.frequenz4; 
    }
    return retValue;
  }
}