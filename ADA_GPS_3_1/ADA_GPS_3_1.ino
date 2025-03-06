

/*                 


----------------------------------------------ReadOut--------------------------------------------------- 			
					DatiVolo		Nextion						MaxDati			Nextion			 					StatDati		Nextion		
					Type			Page1					    Type			Page2				                Page3			

Date	      		  string			t11			maxalt 			    float	%.2f	n2				MaxThermTime		string		
Time				      string						  maxspeed		    float	%.2f	n3				TotThermTime		string		
FieldPressure		  float	%.2f				  maxvario 		    float	%.2f	x21				maxCanopyTemp		float	%.2f	
FieldTemperature 	float	%.2f			 	  course 			    float	%.2f	x20				MaxGainedAltitude	float	%.2f	
CanopyTemp 			  float	%.2f				  durationBuffer	string			t20				TotGainedAltitude	float	%.2f	
currentAltitude 	float	%.2f	n0			FlightStart		  string			t21				
currentSpeed		  float	%.2f	n1			EndFlight		    string			t22				
vario				      float	%.2f	x10			Date			      string			t29				
SingleThermTime		string												
GainedAltitude		float	%.2f											
													
longitude		 	    not in use												
latitude			    not in use												
GPS.satellites		not in use												

Adjust VARIO_THRESHOLD based on your specific glider and typical thermal strengths
Modify STABLE_DURATION to match your typical thermal entry/exit characteristics
Experiment with alpha to find the right balance between smoothness and responsiveness

correggere maxTime in Thermal come stringa anche su nextion
 
Da controllare :
-ElapsedTimeInThermal=0; per verificare se calcola il tempo della singola termica. VARIO_THRESHOLD da aumentare ,alpha da diminuire	 

(GPS.day), (GPS.month), (GPS.year),GPS.hour + 1, GPS.minute, GPS.seconds,FieldPressure, FieldTemperature, CanopyTemp, currentAltitude, currentSpeed, vario, SingleThermTime.c_str(), TotThermTime.c_str()) 
 (GPS.day), (GPS.month), (GPS.year),GPS.hour + 1, GPS.minute, GPS.seconds,GPS.longitude, GPS.latitude,currentAltitude, currentSpeed, GPS.satellites, vario);  //(int)GPS.speed  
 
 maxalt, maxspeed, maxvario, course,durationBuffer, startBuffer, endBuffer, Date.c_str(),maxTimeInThermal, TotalThermalFlight ,maxCanopyTemp,MaxGainedAltitude);
  maxalt, maxspeed, maxvario, course, durationBuffer, FlightStart.c_str(), EndFlight.c_str(), Date.c_str())
  */


#include <SD.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <Adafruit_MPL3115A2.h>

#define SD_CS_PIN 4  // Define pins for the SD card and GPS

// Initialize GPS and barometer
Adafruit_GPS GPS(&Serial1);
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

uint32_t timer = millis();
String SingleThermTime = "";
String TotThermTime = "";
String MaxThermTime = "";
String folderName;
String datiVoloPath;
String maxDatiPath;
String StatDatiPath;
String FlightStart = "";
String EndFlight = "";
String Date = "";
float FieldPressure = 0.00; 
float FieldTemperature =0.00;
float maxCanopyTemp, currentTime;
float previousAltitude = 0.0;
float previousTime = 0.0;
float vario = 0.00;
float maxalt = 0.00;
float maxspeed = 0.00;
float maxvario = 0.00;
float course = 0.00;
float ElapsedTimeInThermal = 0.0;  // Elapsed time with positive vario
float TotalThermalFlight;
float StartAltitude = 0.0;
float EndAltitude = 0.0;
float GainedAltitude = 0.0;
float MaxGainedAltitude = 0.0;
float TotGainedAltitude = 0.0;
float maxTimeInThermal = 0.00;  // Maximum time in single thermal
char durationBuffer[20];
char startBuffer[20];
char endBuffer[20];

int FlightStartTime = 0;
int FlightEndTime = 0;

bool isPositivePeriod = false;
bool flightStarted = false;

static float previousVario = 0;
const float VARIO_THRESHOLD = 0.1;           // Adjust this threshold as needed
const unsigned long STABLE_DURATION = 1000;  // 1 seconds stability requirement

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(8, OUTPUT);
  // Initialize serial communication
  Serial.begin(115200);
  Serial1.begin(9600);
  // Initialize the SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Errore nell'inizializzazione della scheda SD");
    return;
  }
  Serial.println("Scheda SD inizializzata");
  // Initialize GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(200);
  // Initialize barometer
  if (!baro.begin()) {
    Serial.println("Barometer initialization failed!");
    while (1)
      ;
  }
 // baro.setSeaPressure(1025);  // Set sea level pressure (e.g., 1013.25 Pa for standard sea level pressure)
  calibrateSensors();
  // Wait for the GPS to get a fix
  while (!GPS.fix) {
    char c = GPS.read();
    digitalWrite(LED_BUILTIN, LOW);
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) {
        digitalWrite(LED_BUILTIN, HIGH);
        continue;
      }
    }
    digitalWrite(LED_BUILTIN, HIGH);
  }
  // Create the folder and files
  createFolderAndOpenFiles();
  Date = String(GPS.day) + "/" + String(GPS.month) + "/" + String(GPS.year);
  delay(3000);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  while (Serial1.available()) {
    // Read data from the GPS
    char c = GPS.read();
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) {
        return;
      }
      if (millis() - timer > 300) {  // Manage the write frequency with millis()
        timer = millis();            // reset the timer
        digitalWrite(8, HIGH);       // turn the LED on (HIGH is the voltage level)
        digitalWrite(LED_BUILTIN, LOW);
        // Define files
        File datiVoloFile;
        File maxDatiFile;
        File StatDatiFile;
        // Read data from the barometer and the GPS
        float currentAltitude = baro.getAltitude();
        float CanopyTemp = baro.getTemperature();  // temeperature inside fuselage
        float currentSpeed = (GPS.speed * 1852) / 1000;
        currentTime = calculateCurrentTimeFromGPS(GPS);                                                              // Calculate current time in seconds from GPS data
        calculateVario(currentAltitude, currentTime, currentSpeed);                                                  // Call calculateVario with the current altitude and current time
        calculateFlightTimesDuration(currentTime);                                                                   // Calculate flight finish & duration
        calculateMaxValues(currentAltitude, currentSpeed, vario, ElapsedTimeInThermal, CanopyTemp, GainedAltitude);  // Update max values
        // Update DatiVolo
        if (datiVoloFile = SD.open(datiVoloPath.c_str(), FILE_WRITE)) {
          char buffer[200];
          sprintf(buffer, " %s, %02d:%02d:%02d, %.2f, %.2f, %.2f, %.2f, %s,%.2f",
                  Date.c_str(),
                  GPS.hour + 1, GPS.minute, GPS.seconds,
                   CanopyTemp, currentAltitude, currentSpeed, vario, SingleThermTime.c_str(), GainedAltitude);  // ElapsedTimeInThermal, TotalThermalFlight);
          datiVoloFile.println(buffer);
          datiVoloFile.close();
        } else {
          Serial.println("Errore nella scrittura del file DatiVolo.txt");
        }
        // Update MaxDati
        if (maxDatiFile = SD.open(maxDatiPath.c_str(), O_RDWR)) {
          char maxBuffer[200];  // Overwrite the max values in the file     ______________ correggere maxTime in Thermal come stringa anche su nextion
          sprintf(maxBuffer, "%.2f, %.2f, %.2f, %.2f, %s, %s, %s, %s",
                  maxalt, maxspeed, maxvario, course,
                  durationBuffer, startBuffer, endBuffer, Date.c_str());
          maxDatiFile.print(maxBuffer);  // Use print instead of println to avoid adding a new line
          maxDatiFile.close();
        } else {
          Serial.println("Errore nella scrittura del file MaxDati.txt");
        }
        // Update StatDati
        if (StatDatiFile = SD.open(StatDatiPath.c_str(), O_RDWR)) {
          char StatDatiBuffer[200];
          sprintf(StatDatiBuffer, "%s,%s,%.2f,%.2f,%.2f,%.2f,%.2f",
                  MaxThermTime.c_str(), TotThermTime.c_str(),
                  maxCanopyTemp, MaxGainedAltitude, TotGainedAltitude, FieldPressure, FieldTemperature);
          StatDatiFile.print(StatDatiBuffer);
          StatDatiFile.close();
        } else {
          Serial.println("Errore nella scrittura del file StatDati.txt");
        }
        digitalWrite(8, LOW);  // turn the LED off
        digitalWrite(LED_BUILTIN, HIGH);
      }
    }
  }
}
//---------------------------------------------------------------------------Functions-------------------------------------------------------------------------------------------
// Perform initial calibration of barometer
void calibrateSensors() {
  FieldPressure = baro.getPressure();
  FieldTemperature = baro.getTemperature();
  baro.setSeaPressure(FieldPressure); // per misurare la altezza relativa
}
// Function to calculate current time in seconds from GPS data
float calculateCurrentTimeFromGPS(Adafruit_GPS &gps) {
  currentTime = (GPS.hour * 3600.0) + (GPS.minute * 60.0) + GPS.seconds + (GPS.milliseconds / 1000.0);
  return currentTime;
}
// Function to create folder with DateTime name from GPS data
void createFolderAndOpenFiles() {
  String paddedMonth = String(GPS.month);
  if (GPS.month < 10) paddedMonth = "0" + paddedMonth;

  String paddedDay = String(GPS.day);
  if (GPS.day < 10) paddedDay = "0" + paddedDay;

  String paddedHour = String(GPS.hour + 1);  // Adding 1 as per original code
  if ((GPS.hour + 1) < 10) paddedHour = "0" + paddedHour;

  String paddedMinute = String(GPS.minute);
  if (GPS.minute < 10) paddedMinute = "0" + paddedMinute;

  // Combine the padded values
  folderName = paddedDay + paddedMonth + paddedHour + paddedMinute;
  if (!SD.exists(folderName.c_str())) {
    if (SD.mkdir(folderName.c_str())) {
      Serial.println("Cartella creata con successo: " + folderName);
    } else {
      Serial.println("Errore nella creazione della cartella: " + folderName);
    }
  } else {
    Serial.println("La cartella esiste già: " + folderName);
  }
  datiVoloPath = folderName + "/DatiVolo.txt";  // Create the files inside the folder
  maxDatiPath = folderName + "/MaxDati.txt";
  StatDatiPath = folderName + "/StatDati.txt";
  File datiVoloFile;
  File maxDatiFile;
  File StatDatiFile;
  if (datiVoloFile = SD.open(datiVoloPath.c_str(), FILE_WRITE)) {
    datiVoloFile.close();
  } else {
    Serial.println("Errore nella creazione del file DatiVolo.txt");
  }
  if (maxDatiFile = SD.open(maxDatiPath.c_str(), FILE_WRITE)) {
    maxDatiFile.close();
  } else {
    Serial.println("Errore nella creazione del file MaxDati.txt");
  }
  if (StatDatiFile = SD.open(StatDatiPath.c_str(), FILE_WRITE)) {
    StatDatiFile.close();
  } else {
    Serial.println("Errore nella creazione del file StatDati.txt");
  }
}
// Function to calculate flight start, end, and duration
void calculateFlightTimesDuration(float currentTime) {
  if (!flightStarted) {
    sprintf(startBuffer, "%02d:%02d:%02d", GPS.hour + 1, GPS.minute, GPS.seconds);
    FlightStartTime = ((GPS.hour * 3600.0) + (GPS.minute * 60.0) + GPS.seconds);  // in seconds
    flightStarted = true;
  }
  sprintf(endBuffer, "%02d:%02d:%02d", GPS.hour + 1, GPS.minute, GPS.seconds);
  FlightEndTime = ((GPS.hour * 3600.0) + (GPS.minute * 60.0) + GPS.seconds);
  int durationSeconds = FlightEndTime - FlightStartTime;
  int hours = durationSeconds / 3600;
  int minutes = (durationSeconds % 3600) / 60;
  int seconds = durationSeconds % 60;
  sprintf(durationBuffer, "%02d:%02d:%02d", hours, minutes, seconds);
}
// Calculate the rate of altitude change in m/s
void calculateVario(float currentAltitude, float currentTime, float currentSpeed) {
  static unsigned long lastStableTime = 0;
  float timeChange;
  if (previousTime != 0 && currentTime > previousTime + 0.8) {  // Avoid division by zero and ensure meaningful time interval. Ptrev TYime is for first run only
    float altitudeChange = currentAltitude - previousAltitude;
    timeChange = currentTime - previousTime;
    float instantVario = altitudeChange / timeChange;  // Calculate vario with smoothing
    //  low-pass filter Lower values (closer to 0) create more smoothing Higher values (closer to 1) make the response more responsive.
    const float alpha = 0.9;  // Adjust this value to control smoothing
    vario = (alpha * instantVario) + ((1 - alpha) * previousVario);
    previousVario = vario;
    if (currentSpeed>10) {
    float travel = currentSpeed * timeChange;  // Calculate travel and course; travel to manage descendant and negative values
    course += abs(travel);
    }
  //  float travel = currentSpeed * timeChange;  // Calculate travel and course; travel to manage descendant and negative values
    //course += abs(travel);
  }
  // Thermal detection logic with hysteresis Introduces a threshold to distinguish meaningful vertical movement
  if (vario > VARIO_THRESHOLD) {  // Only starts timing when vario has been consistently positive.
    if (!isPositivePeriod) {
      // Check if vario has been consistently positive
      if (millis() - lastStableTime >= STABLE_DURATION) {
        lastStableTime = millis();
        ElapsedTimeInThermal = 0;
        GainedAltitude = 0;
        StartAltitude = currentAltitude;
        EndAltitude = StartAltitude;
        isPositivePeriod = true;
      }
    } else {
      ElapsedTimeInThermal += (millis() - lastStableTime) / 1000.0;  // Accumulate thermal time
      SingleThermTime = convertTimeToString(ElapsedTimeInThermal);   //write in dati volo
      lastStableTime = millis();
      EndAltitude = currentAltitude;
    }
  } else if (vario < -VARIO_THRESHOLD) {
    if (millis() - lastStableTime >= STABLE_DURATION) {  // Check if vario has been consistently negative
      TotalThermalFlight += ElapsedTimeInThermal;
      TotThermTime = convertTimeToString(TotalThermalFlight);
      GainedAltitude = EndAltitude - StartAltitude;
      TotGainedAltitude += GainedAltitude;
      isPositivePeriod = false;
      ElapsedTimeInThermal = 0;  //
    }
  }
  previousAltitude = currentAltitude;  // Update previous values for next iteration
  previousTime = currentTime;
  previousVario = vario;
}
String convertTimeToString(float timeInSeconds) {
  int minutes = (int)(timeInSeconds / 60);  // Convert seconds to minutes and seconds
  int remainingSeconds = (int)(timeInSeconds - (minutes * 60));
  char timeString[10];  // Format the string
  sprintf(timeString, "%02d:%02d", minutes, remainingSeconds);
  return String(timeString);  // Return the formatted string
}
// Calculate max values of variables
void calculateMaxValues(float currentAltitude, float currentSpeed, float vario, float ElapsedTimeInThermal, float CanopyTemp, float GainedAltitude) {
  if (currentAltitude > maxalt) {
    maxalt = currentAltitude;
  }
  if (currentSpeed > maxspeed) {
    maxspeed = currentSpeed;
  }
  if (vario > maxvario) {
    maxvario = vario;
  }
  if (ElapsedTimeInThermal > maxTimeInThermal) {
    maxTimeInThermal = ElapsedTimeInThermal;
    MaxThermTime = convertTimeToString(maxTimeInThermal);
  }
  if (CanopyTemp > maxCanopyTemp) {
    maxCanopyTemp = CanopyTemp;
  }
  if (GainedAltitude > MaxGainedAltitude) {
    MaxGainedAltitude = GainedAltitude;
  }
}