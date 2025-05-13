

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

// Struttura per i dati delle termiche
struct ThermalData {
  float elapsedTimeInThermal;  // Tempo nella termica corrente
  float totalThermalFlight;    // Tempo totale in termiche
  float maxTimeInThermal;      // Tempo massimo in una singola termica
  float startAltitude;         // Altitudine all'inizio della termica
  float endAltitude;           // Altitudine alla fine della termica
  float gainedAltitude;        // Guadagno di altitudine nella termica corrente
  float maxGainedAltitude;     // Massimo guadagno di altitudine in una singola termica
  float totGainedAltitude;     // Guadagno totale di altitudine
  String singleThermTime;      // Rappresentazione in stringa del tempo nella termica corrente
  String totThermTime;         // Rappresentazione in stringa del tempo totale in termiche
  String maxThermTime;         // Rappresentazione in stringa del tempo massimo in una termica
  bool isPositivePeriod;       // Flag per indicare se siamo in una fase di salita
};

// Struttura per i dati di volo
struct FlightData {
  float currentAltitude;       // Altitudine corrente
  float previousAltitude;      // Altitudine precedente
  float maxAltitude;           // Altitudine massima
  float currentSpeed;          // Velocità corrente
  float maxSpeed;              // Velocità massima
  float vario;                 // Variometro (velocità verticale)
  float previousVario;         // Valore precedente del variometro
  float maxVario;              // Valore massimo del variometro
  float currentTime;           // Tempo corrente calcolato dal GPS
  float previousTime;          // Tempo precedente
  float course;                // Distanza percorsa
  float canopyTemp;            // Temperatura interna
  float maxCanopyTemp;         // Temperatura interna massima
  int flightStartTime;         // Tempo di inizio volo (in secondi)
  int flightEndTime;           // Tempo di fine volo (in secondi)
  bool flightStarted;          // Flag per indicare se il volo è iniziato
  char startBuffer[20];        // Buffer per l'ora di inizio
  char endBuffer[20];          // Buffer per l'ora di fine
  char durationBuffer[20];     // Buffer per la durata del volo
};

// Struttura per i dati ambientali
struct EnvironmentData {
  float fieldPressure;         // Pressione atmosferica al campo
  float fieldTemperature;      // Temperatura al campo
};

// Struttura per i dati del file system
struct FileSystemData {
  String folderName;           // Nome della cartella
  String datiVoloPath;         // Percorso del file DatiVolo.txt
  String maxDatiPath;          // Percorso del file MaxDati.txt
  String statDatiPath;         // Percorso del file StatDati.txt
  String date;                 // Data formattata
  char dateBuffer[20];         // Buffer per la data formattata
};

// Inizializzazione delle strutture
ThermalData thermalData = {
  0.0,        // elapsedTimeInThermal
  0.0,        // totalThermalFlight
  0.0,        // maxTimeInThermal
  0.0,        // startAltitude
  0.0,        // endAltitude
  0.0,        // gainedAltitude
  0.0,        // maxGainedAltitude
  0.0,        // totGainedAltitude
  "",         // singleThermTime
  "",         // totThermTime
  "",         // maxThermTime
  false       // isPositivePeriod
};

FlightData flightData = {
  0.0,        // currentAltitude
  0.0,        // previousAltitude
  0.0,        // maxAltitude
  0.0,        // currentSpeed
  0.0,        // maxSpeed
  0.0,        // vario
  0.0,        // previousVario
  0.0,        // maxVario
  0.0,        // currentTime
  0.0,        // previousTime
  0.0,        // course
  0.0,        // canopyTemp
  0.0,        // maxCanopyTemp
  0,          // flightStartTime
  0,          // flightEndTime
  false,      // flightStarted
  {0},        // startBuffer
  {0},        // endBuffer
  {0}         // durationBuffer
};

EnvironmentData environmentData = {
  0.0,        // fieldPressure
  0.0         // fieldTemperature
};

FileSystemData fileSystemData = {
  "",         // folderName
  "",         // datiVoloPath
  "",         // maxDatiPath
  "",         // statDatiPath
  "",         // date
  {0}         // dateBuffer
};

// Initialize GPS and barometer
Adafruit_GPS GPS(&Serial1);
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

uint32_t timer = millis();

// Costanti
const float VARIO_THRESHOLD = 0.1;            // Soglia per rilevare l'inizio/fine di una termica
const unsigned long STABLE_DURATION = 1000;   // Durata stabilità richiesta (1 secondo)

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(8, OUTPUT);
  
  // Inizializza la comunicazione seriale
  Serial.begin(9600);
  Serial1.begin(9600);
  
  // Inizializza la scheda SD
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Errore nell'inizializzazione della scheda SD");
    return;
  }
  Serial.println("Scheda SD inizializzata");
  
  // Inizializza il GPS
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  delay(200);
  
  // Inizializza il barometro
  if (!baro.begin()) {
    Serial.println("Barometer initialization failed!");
    while (1);
  }
  
  // Calibra i sensori
  calibrateSensors();
  
  // Attendi che il GPS ottenga una fix
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
  
  // Crea le cartelle e i file
  createFolderAndOpenFiles();
  
  // Formatta la data con zeri iniziali
  formatDate();
  
  delay(3000);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  
  while (Serial1.available()) {
    // Leggi dati dal GPS
    char c = GPS.read();
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) {
        return;
      }
      
      if (millis() - timer > 300) {  // Gestisci la frequenza di scrittura con millis()
        timer = millis();            // Reset del timer
        digitalWrite(8, HIGH);       // Accendi il LED
        digitalWrite(LED_BUILTIN, LOW);
        
        // Definisci i file
        File datiVoloFile;
        File maxDatiFile;
        File statDatiFile;
        
        // Leggi dati dal barometro e dal GPS
        flightData.currentAltitude = baro.getAltitude();
        flightData.canopyTemp = baro.getTemperature();  // Temperatura all'interno della fusoliera
        flightData.currentSpeed = (GPS.speed * 1852) / 1000;
        flightData.currentTime = calculateCurrentTimeFromGPS(GPS);  // Calcola il tempo attuale dai dati GPS
        
        // Calcola variometro, durata del volo e valori massimi
        calculateVario();
        calculateFlightTimesDuration();
        calculateMaxValues();
        
        // Aggiorna DatiVolo.txt
        if (datiVoloFile = SD.open(fileSystemData.datiVoloPath.c_str(), FILE_WRITE)) {
          char buffer[200];
          sprintf(buffer, " %s, %02d:%02d:%02d, %.2f, %.2f, %.2f, %.2f, %s, %.2f",
                 fileSystemData.date.c_str(),
                 GPS.hour + 1, GPS.minute, GPS.seconds,
                 flightData.canopyTemp, flightData.currentAltitude, flightData.currentSpeed, flightData.vario, 
                 thermalData.singleThermTime.c_str(), thermalData.gainedAltitude);
          datiVoloFile.println(buffer);
          datiVoloFile.close();
        } else {
          Serial.println("Errore nella scrittura del file DatiVolo.txt");
        }
        
        // Aggiorna MaxDati.txt
        if (maxDatiFile = SD.open(fileSystemData.maxDatiPath.c_str(), O_RDWR)) {
          char maxBuffer[200];
          sprintf(maxBuffer, "%.2f, %.2f, %.2f, %.2f, %s, %s, %s, %s",
                 flightData.maxAltitude, flightData.maxSpeed, flightData.maxVario, flightData.course,
                 flightData.durationBuffer, flightData.startBuffer, flightData.endBuffer, fileSystemData.date.c_str());
          maxDatiFile.print(maxBuffer);  // Usa print invece di println per evitare di aggiungere una nuova riga
          maxDatiFile.close();
        } else {
          Serial.println("Errore nella scrittura del file MaxDati.txt");
        }
        
        // Aggiorna StatDati.txt
        if (statDatiFile = SD.open(fileSystemData.statDatiPath.c_str(), O_RDWR)) {
          char statDatiBuffer[200];
          // Versione migliorata con spazi dopo le virgole (coerente con MaxDati.txt)
          sprintf(statDatiBuffer, "%s, %s, %.2f, %.2f, %.2f, %.2f, %.2f",
                 thermalData.maxThermTime.c_str(), thermalData.totThermTime.c_str(),
                 flightData.maxCanopyTemp, thermalData.maxGainedAltitude, thermalData.totGainedAltitude, 
                 environmentData.fieldPressure, environmentData.fieldTemperature);
          statDatiFile.print(statDatiBuffer);
          statDatiFile.close();
        } else {
          Serial.println("Errore nella scrittura del file StatDati.txt");
        }
        
        digitalWrite(8, LOW);  // Spegni il LED
        digitalWrite(LED_BUILTIN, HIGH);
      }
    }
  }
}

//---------------------------------------------------------------------------Funzioni-------------------------------------------------------------------------------------------

// Calibrazione iniziale del barometro
void calibrateSensors() {
  environmentData.fieldPressure = baro.getPressure();
  environmentData.fieldTemperature = baro.getTemperature();
  baro.setSeaPressure(environmentData.fieldPressure);
  float initialAltitude = baro.getAltitude();
  if (initialAltitude != 0.0) {
    float correctedSeaPressure = environmentData.fieldPressure * pow(1 - (initialAltitude / 44330.0), 5.255);
    baro.setSeaPressure(correctedSeaPressure);
  }
}

// Calcola il tempo corrente in secondi dai dati GPS
float calculateCurrentTimeFromGPS(Adafruit_GPS &gps) {
  return (GPS.hour * 3600.0) + (GPS.minute * 60.0) + GPS.seconds + (GPS.milliseconds / 1000.0);
}

// Formatta correttamente la data
void formatDate() {
  // Formatta la data con zeri iniziali per mese e giorno
  snprintf(fileSystemData.dateBuffer, sizeof(fileSystemData.dateBuffer), "%02d/%02d/%02d", 
           GPS.day, 
           GPS.month, 
           GPS.year);
  fileSystemData.date = String(fileSystemData.dateBuffer);
}

// Crea la cartella con il nome DateTime dai dati GPS
void createFolderAndOpenFiles() {
  String paddedMonth = String(GPS.month);
  if (GPS.month < 10) paddedMonth = "0" + paddedMonth;

  String paddedDay = String(GPS.day);
  if (GPS.day < 10) paddedDay = "0" + paddedDay;

  String paddedHour = String(GPS.hour + 1);  // Aggiunta 1 come nel codice originale
  if ((GPS.hour + 1) < 10) paddedHour = "0" + paddedHour;

  String paddedMinute = String(GPS.minute);
  if (GPS.minute < 10) paddedMinute = "0" + paddedMinute;

  // Combina i valori con padding
  fileSystemData.folderName = paddedDay + paddedMonth + paddedHour + paddedMinute;
  
  if (!SD.exists(fileSystemData.folderName.c_str())) {
    if (SD.mkdir(fileSystemData.folderName.c_str())) {
      Serial.println("Cartella creata con successo: " + fileSystemData.folderName);
    } else {
      Serial.println("Errore nella creazione della cartella: " + fileSystemData.folderName);
    }
  } else {
    Serial.println("La cartella esiste già: " + fileSystemData.folderName);
  }
  
  // Crea i file all'interno della cartella
  fileSystemData.datiVoloPath = fileSystemData.folderName + "/DatiVolo.txt";
  fileSystemData.maxDatiPath = fileSystemData.folderName + "/MaxDati.txt";
  fileSystemData.statDatiPath = fileSystemData.folderName + "/StatDati.txt";
  
  File datiVoloFile, maxDatiFile, statDatiFile;
  
  if (datiVoloFile = SD.open(fileSystemData.datiVoloPath.c_str(), FILE_WRITE)) {
    datiVoloFile.close();
  } else {
    Serial.println("Errore nella creazione del file DatiVolo.txt");
  }
  
  if (maxDatiFile = SD.open(fileSystemData.maxDatiPath.c_str(), FILE_WRITE)) {
    maxDatiFile.close();
  } else {
    Serial.println("Errore nella creazione del file MaxDati.txt");
  }
  
  if (statDatiFile = SD.open(fileSystemData.statDatiPath.c_str(), FILE_WRITE)) {
    statDatiFile.close();
  } else {
    Serial.println("Errore nella creazione del file StatDati.txt");
  }
}

// Calcola inizio, fine e durata del volo
void calculateFlightTimesDuration() {
  if (!flightData.flightStarted) {
    sprintf(flightData.startBuffer, "%02d:%02d:%02d", GPS.hour + 1, GPS.minute, GPS.seconds);
    flightData.flightStartTime = ((GPS.hour * 3600.0) + (GPS.minute * 60.0) + GPS.seconds);  // in secondi
    flightData.flightStarted = true;
  }
  
  sprintf(flightData.endBuffer, "%02d:%02d:%02d", GPS.hour + 1, GPS.minute, GPS.seconds);
  flightData.flightEndTime = ((GPS.hour * 3600.0) + (GPS.minute * 60.0) + GPS.seconds);
  
  int durationSeconds = flightData.flightEndTime - flightData.flightStartTime;
  int hours = durationSeconds / 3600;
  int minutes = (durationSeconds % 3600) / 60;
  int seconds = durationSeconds % 60;
  
  sprintf(flightData.durationBuffer, "%02d:%02d:%02d", hours, minutes, seconds);
}

// Calcola il tasso di variazione dell'altitudine in m/s
void calculateVario() {
  static unsigned long lastStableTime = 0;
  static float previousGoodAltitude = 0; // Aggiungi questa variabile
  float timeChange;
  
  // Evita divisione per zero e assicura un intervallo di tempo significativo
  if (flightData.previousTime != 0 && flightData.currentTime > flightData.previousTime + 0.8) {
    float altitudeChange = flightData.currentAltitude - flightData.previousAltitude;
    timeChange = flightData.currentTime - flightData.previousTime;
    
    // Calcola il vario con smoothing
    float instantVario = altitudeChange / timeChange;
    // Filtro passa-basso: valori più bassi (vicino a 0) creano più smoothing, valori più alti (vicino a 1) rendono la risposta più reattiva
    const float alpha = 0.85;  // Regola questo valore per controllare lo smoothing
    flightData.vario = (alpha * instantVario) + ((1 - alpha) * flightData.previousVario);
    flightData.previousVario = flightData.vario;
    
    // Calcola il percorso solo se la velocità è significativa
    if (flightData.currentSpeed > 2) {
      float travel = flightData.currentSpeed * timeChange;
      flightData.course += abs(travel);
    }
  }
  
  // Logica di rilevamento termiche con isteresi
  // Introduce una soglia per distinguere movimenti verticali significativi
  if (flightData.vario > VARIO_THRESHOLD) {  // Inizia a misurare solo quando il vario è consistentemente positivo
    if (!thermalData.isPositivePeriod) {
      if (millis() - lastStableTime >= STABLE_DURATION) {
        lastStableTime = millis();
        thermalData.elapsedTimeInThermal = 0;
        thermalData.gainedAltitude = 0;
        thermalData.startAltitude = flightData.currentAltitude;
        thermalData.endAltitude = thermalData.startAltitude;
        thermalData.isPositivePeriod = true;
        previousGoodAltitude = flightData.currentAltitude; // Memorizza la quota iniziale
      }
    } else {
      // Accumula il tempo in termica
      thermalData.elapsedTimeInThermal += (millis() - lastStableTime) / 1000.0;
      thermalData.singleThermTime = convertTimeToString(thermalData.elapsedTimeInThermal);
      lastStableTime = millis();
      thermalData.endAltitude = flightData.currentAltitude;
    }
  } else if (flightData.vario < -VARIO_THRESHOLD) {
    // Verifica se il vario è stato consistentemente negativo
    if (millis() - lastStableTime >= STABLE_DURATION) {
      thermalData.totalThermalFlight += thermalData.elapsedTimeInThermal;
      thermalData.totThermTime = convertTimeToString(thermalData.totalThermalFlight);
      
      // Calcola il guadagno SOLO se positivo
      float currentGain = flightData.currentAltitude - previousGoodAltitude;
      if (currentGain > 0) {
        thermalData.totGainedAltitude += currentGain;
      }
      
      previousGoodAltitude = flightData.currentAltitude; // Aggiorna per la prossima termica
      thermalData.isPositivePeriod = false;
      thermalData.elapsedTimeInThermal = 0;
    }
  }
  
  // Aggiorna i valori precedenti per la prossima iterazione
  flightData.previousAltitude = flightData.currentAltitude;
  flightData.previousTime = flightData.currentTime;
}

// Converte il tempo in secondi in una stringa formattata
String convertTimeToString(float timeInSeconds) {
  int minutes = (int)(timeInSeconds / 60);
  int remainingSeconds = (int)(timeInSeconds - (minutes * 60));
  char timeString[10];
  sprintf(timeString, "%02d:%02d", minutes, remainingSeconds);
  return String(timeString);
}

// Calcola i valori massimi delle variabili
void calculateMaxValues() {
  // Aggiorna l'altitudine massima
  if (flightData.currentAltitude > flightData.maxAltitude) {
    flightData.maxAltitude = flightData.currentAltitude;
  }
  
  // Aggiorna la velocità massima
  if (flightData.currentSpeed > flightData.maxSpeed) {
    flightData.maxSpeed = flightData.currentSpeed;
  }
  
  // Aggiorna il vario massimo
  if (flightData.vario > flightData.maxVario) {
    flightData.maxVario = flightData.vario;
  }
  
  // Aggiorna il tempo massimo in termica
  if (thermalData.elapsedTimeInThermal > thermalData.maxTimeInThermal) {
    thermalData.maxTimeInThermal = thermalData.elapsedTimeInThermal;
    thermalData.maxThermTime = convertTimeToString(thermalData.maxTimeInThermal);
  }
  
  // Aggiorna la temperatura massima della calotta
  if (flightData.canopyTemp > flightData.maxCanopyTemp) {
    flightData.maxCanopyTemp = flightData.canopyTemp;
  }
  
  // Aggiorna il guadagno massimo di altitudine
  if (thermalData.gainedAltitude > thermalData.maxGainedAltitude) {
    thermalData.maxGainedAltitude = thermalData.gainedAltitude;
  }
}