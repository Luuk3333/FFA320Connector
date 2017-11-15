/*
* FFA320-Connector by mokny
*
* This is a plugin for the FlightFactor A320 Ultimate
* It allows you to map any internal A320 variable to a command or Dataref.
*
*/

#pragma warning(disable: 4996)

#include "XPLMDataAccess.h"
#include "XPLMPlugin.h"
#include "XPLMUtilities.h"
#include "XPLMProcessing.h"
#include "XPLMMenus.h"
#include "XPLMPlanes.h"
#include "XPLMDataAccess.h"
#include <stdio.h>
#include <algorithm>
#include <string.h>
#include <string>
#include <math.h>
#include <list>
#include <string>
#include <iostream>
#include <fstream>
#include "SharedValue.h"

using namespace std;

string					pluginversion = "1.0.8";																			// Plugin-Version

#define					XPLM200 = 1;																						// SDK 2 Version
#define					MSG_ADD_DATAREF 0x01000000																			// Add dataref to DRE message

const int				OBJECT_TYPE_COMMAND = 1;																			// Command Object
const int				OBJECT_TYPE_DATAREF = 2;																			// Dataref Object
const int				VALUE_TYPE_INT = 1;																					// Integer Value														
const int				VALUE_TYPE_FLOAT = 2;																				// Float Value
const int				WORK_MODE_SET = 1;																					// Workmode Definitions
const int				WORK_MODE_STEP = 2;
const int				WORK_MODE_CYCLE = 3;
const int				WORK_MODE_CLICK = 4;
const int				WORK_MODE_ROTATE = 5;

bool					plugindisabled = FALSE;																				// True if plugin is disabled
bool					plugininitialized = FALSE;																			// Plugin Initialized? Set when Flightloop was called.
XPLMPluginID			ffPluginID = XPLM_NO_PLUGIN_ID;
SharedValuesInterface	ffAPI;
int						g_menu_container_idx;																				// Menu Stuff
XPLMMenuID				g_menu_id;																							// The menu container we'll append all our menu items to
void					menu_handler(void *, void *);
int						ffAPIdataversion = 0;
bool					debugmode = FALSE;																					// Enable extensive logging?
void*					tag = "ffa320connector";																			// ffAPI Tag (shall we change this?!)
double					last_step;
float					PluginCustomFlightLoopCallback(float elapsedMe, float elapsedSim, int counter, void * refcon);		// FlightLoop, only called once for init
int						UniversalCommandHandler(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void * inRefcon);		// Handles all Commands
int						UniversalDataRefGET_INT(void* inRefcon);															// Handles all DataRef GET-Requests for Integer-Values
void					UniversalDataRefSET_INT(void* inRefcon, int inValue);												// Handles all DataRef SET-Requests for Integer-Values
float					UniversalDataRefGET_FLOAT(void* inRefcon);															// Handles all DataRef GET-Requests for Integer-Values
void					UniversalDataRefSET_FLOAT(void* inRefcon, float inValue);											// Handles all DataRef SET-Requests for Integer-Values
int						DrefValueInt[2000];																					// Stores the Dataref-Values (inRefcon points to here)
float					DrefValueFloat[2000];																				// Stores the Dataref-Values (inRefcon points to here)
bool					InternalDatarefUpdate = FALSE;																		// For recognition if it was an internal Dref update or not

bool					DumpObjectsToLogActive = FALSE;
void					DumpObjectsToLog();																					//Constructor

/*
* StringToObjectType
*
* Converts a string to the defined Object-Type
*
*/
int StringToObjectType(string s) {
	transform(s.begin(), s.end(), s.begin(), ::toupper);
	if (s == "COMMAND") return OBJECT_TYPE_COMMAND;
	if (s == "DATAREF") return OBJECT_TYPE_DATAREF;
	return 0;
}

/*
* StringToValueType
*
* Converts a string to the defined Value-Type
*
*/
int StringToValueType(string s) {
	transform(s.begin(), s.end(), s.begin(), ::toupper);
	if (s == "INT") return VALUE_TYPE_INT;
	if (s == "FLOAT") return VALUE_TYPE_FLOAT;
	return 0;
}

/*
* StringToWorkMode
*
* Converts a string to the defined Work-Mode
*
*/
int StringToWorkMode(string s) {
	transform(s.begin(), s.end(), s.begin(), ::toupper);
	if (s == "SET") return WORK_MODE_SET;
	if (s == "STEP") return WORK_MODE_STEP;
	if (s == "CYCLE") return WORK_MODE_CYCLE;
	if (s == "CLICK") return WORK_MODE_CLICK;
	if (s == "ROTATE") return WORK_MODE_ROTATE;
	return 0;
}

/*
* DebugOut
*
* Writes lines to the Log.txt if debugging was enabled
*
*/
void DebugOut(string text) {
	if (debugmode == true) {
		string line = "FFA320Connector DEBUG: " + text + "\n";
		XPLMDebugString(line.c_str());
	}
}

/*
* LogWrite
*
* Writes lines to the Log.txt - no matter if debugging is on or off
*
*/
void LogWrite(string text) {
	string line = "FFA320Connector: " + text + "\n";
	XPLMDebugString(line.c_str());
}

/*
* file_exists
*
* Checks if a file exists
*
*/
inline bool file_exists(const std::string& name) {
	ifstream f(name.c_str());
	return f.good();
}

/*
* DataObject
*
* Main Data-Object for Commands and Datarefs
*
*/
class DataObject {

	public:
		string		FFVar;					// FlightFactor - Object
		int			FFID;					// Object ID
		int			Type;					// COMMAND or DATAREF
		int			WorkMode;				// SET/CYCLE/STEP/CLICK
		int			ValueType;				// INT
		string		Command;				// Command-Identifier
		string		CommandName;			// Command Name
		string		FFReference;			// FlightFactor Reference Object of increment/decrement
		int			FFReferenceID;			// FlightFactor Reference ID
		string		DataRef;				// Dataref
		int			DataRefValueType = 0;	// INT,FLOAT
		bool		IsExistingDataRef;		// Is this a custom or foreign Dataref?
		bool		IgnoreExistingDataRef;	// Ignore existing DataRef and create Handlers anyway
		int			Value;					// Value or Change-Value for COMMAND
		float		ValueFloat;				// Float Value for COMMAND
		int			MinValue;				// MinValue for CYCLE, STEP -- or Reset-Value for CLICK
		int			MaxValue;				// MaxValue for CYCLE, STEP 
		float		MinValueFloat;			// MinValueFloat for CYCLE, STEP -- or Reset-Value for CLICK
		float		MaxValueFloat;			// MaxValueFloat for CYCLE, STEP 
		bool		Cycle;					// Cycle?
		bool		NeedsUpdate;			// Tells the Flightloop if this object needs to be updated
		bool		NeedsClickUpdate;		// After a Click, this must be true
		int			ClickTimer;				// How many cycles before resetting to MinValue?
		int			SpeedRef;				// How fast increment/Decrement STEP/CYCLE
		int			NextUpdateCycle;		// Internal
		int			Phase;					// Button-Phase
		int			RefConID;				// RefconID - The link between the DREF and UniversalGET
		float		DataRefMultiplier;		// Dataref Multiplier

		int*		pAdress;				// Pointer to the Integer Refcon
		float*		pAdressf;				// Pointer to the Float Refcon

		XPLMCommandRef	CMD = NULL;			// The command 
		XPLMDataRef		DREF = NULL;		// The dataref

		void initialize() {
			/* Create the command */
			if (Type == OBJECT_TYPE_COMMAND) {
				LogWrite("Creating Command " + Command + " / " + to_string(WorkMode) + " / " + FFVar);
				CMD = XPLMCreateCommand(Command.c_str(), CommandName.c_str());
				XPLMRegisterCommandHandler(CMD, UniversalCommandHandler, 1, &Value);
				NextUpdateCycle = 0;
			}

			/* Create the dataref */
			if (Type == OBJECT_TYPE_DATAREF) {

				if (DataRefValueType < 1) DataRefValueType = ValueType;				// If no ValueType is set in the config, use the default one

				DREF = XPLMFindDataRef(DataRef.c_str());							// Check if the Dataref already exists

				if ((DREF == NULL) || (IgnoreExistingDataRef == true)) {
					IsExistingDataRef = false;
					LogWrite("Creating Dataref " + DataRef + " / " + to_string(ValueType) + " / #" + to_string(RefConID) + " / " + FFVar);
					if (DataRefValueType == VALUE_TYPE_INT) {
						DREF = XPLMRegisterDataAccessor(DataRef.c_str(),
							xplmType_Int,											// The types we support
							1,														// Writable
							UniversalDataRefGET_INT, UniversalDataRefSET_INT,		// Integer accessors
							NULL, NULL,												// Float accessors
							NULL, NULL,												// Doubles accessors
							NULL, NULL,												// Int array accessors
							NULL, NULL,												// Float array accessors
							NULL, NULL,												// Raw data accessors
							&DrefValueInt[RefConID], &DrefValueInt[RefConID]);      // Refcons				
						pAdress = &DrefValueInt[RefConID];
					}
					if (DataRefValueType == VALUE_TYPE_FLOAT) {
						DREF = XPLMRegisterDataAccessor(DataRef.c_str(),
							xplmType_Float,											// The types we support
							1,														// Writable
							NULL, NULL,												// Integer accessors
							UniversalDataRefGET_FLOAT, UniversalDataRefSET_FLOAT,   // Float accessors
							NULL, NULL,												// Doubles accessors
							NULL, NULL,												// Int array accessors
							NULL, NULL,												// Float array accessors
							NULL, NULL,												// Raw data accessors
							&DrefValueFloat[RefConID], &DrefValueFloat[RefConID]);  // Refcons 				
						pAdressf = &DrefValueFloat[RefConID];
					}

					/*Report the Dataref to the Datarefeditor (This will only worke at this point, if the plugin resides in the
					  Aircraft's Plugin Directory. Otherwise the DRE may not be loaded here. That's why we do it twice... */

					XPLMPluginID PluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
					if ((PluginID != XPLM_NO_PLUGIN_ID)) XPLMSendMessageToPlugin(PluginID, MSG_ADD_DATAREF, (void*)DataRef.c_str());

				} else {
					IsExistingDataRef = true;
					LogWrite("Using existing Dataref (READONLY) " + DataRef + " / " + to_string(ValueType) + " / #" + to_string(RefConID) + " / " + FFVar);
				}

				NextUpdateCycle = 0;
			}
		}
		
		void destroy() {
			/* Remove the CommandHandler + Dataref */
			if (CMD != NULL) XPLMUnregisterCommandHandler(CMD, UniversalCommandHandler, 0, 0);
			if (DREF != NULL) XPLMUnregisterDataAccessor(DREF);
		}

};



list<DataObject>	DataObjects;													// This list contains all Data-Objects
void				ReadIni();														// Constructor 
void				ffAPIUpdateCallback(double step, void *tag);					// FFAPI Constructor




/*
* XPluginStart
*
* Our start routine registers our window and does any other initialization we
* must do.
*
*/
PLUGIN_API int XPluginStart(
	char *		outName,
	char *		outSig,
	char *		outDesc)
{
	strcpy(outName, "FFA320-Connector");
	strcpy(outSig, "mokny.a320connector");
	strcpy(outDesc, "Plugin to supply Commands and Datarefs for the FlightFactor A320");

	string menu_title = string("FFA320-Connector " + pluginversion);

	/* Menu Stuff */
	g_menu_container_idx = XPLMAppendMenuItem(XPLMFindPluginsMenu(), menu_title.c_str(), 0, 0);
	g_menu_id = XPLMCreateMenu(menu_title.c_str(), XPLMFindPluginsMenu(), g_menu_container_idx, menu_handler, NULL);
	XPLMAppendMenuItem(g_menu_id, "Reload Config", (void *)"Reload Config", 1);
	XPLMAppendMenuSeparator(g_menu_id);
	XPLMAppendMenuItem(g_menu_id, "Debug-Logging On/Off", (void *)"Debug-Logging On/Off", 1);
	XPLMAppendMenuItem(g_menu_id, "Dump Objects to Log.txt", (void *)"Dump Objects to Log.txt", 1);

	/* Initial Load */
	LogWrite("==== FFA320 Connector loaded - Version " + pluginversion + " by mokny ====");

	/* Read the Config */
	ReadIni();

	/* Register the Flightloop */
	XPLMRegisterFlightLoopCallback(PluginCustomFlightLoopCallback, 1, NULL); 

	/* We are not disabled, right? */
	plugindisabled = false;

	return 1;
}

/*
* menu_handler
*
* Handles our Menu-Clicks
*
*/
void menu_handler(void * in_menu_ref, void * in_item_ref)
{
	/* Leave if plugin was disabled */
	if (plugindisabled == true) return;

	if (!strcmp((const char *)in_item_ref, "Reload Config"))
	{
		LogWrite("==== FFA320 Connector / Reloaded Config ====");
		ReadIni();
	}
	if (!strcmp((const char *)in_item_ref, "Debug-Logging On/Off"))
	{
		if (debugmode) {
			LogWrite("==== FFA320 Connector / Debug-Logging DISABLED ====");
			debugmode = false;
		}
		else {
			LogWrite("==== FFA320 Connector / Debug-Logging ENABLED ====");
			debugmode = true;
		}
	}
	if (!strcmp((const char *)in_item_ref, "Dump Objects to Log.txt"))
	{
		DumpObjectsToLogActive = true;
	}
}

/*
* UniversalCommandHandler
*
* Handles all incoming Commands and updates the respective DataObject
*
*/
int UniversalCommandHandler(XPLMCommandRef inCommand, XPLMCommandPhase inPhase, void * inRefcon)
{
	list<DataObject>::iterator  iDataObjects;

	for (iDataObjects = DataObjects.begin(); iDataObjects != DataObjects.end(); ++iDataObjects) {
		if (inCommand == iDataObjects->CMD) {
			if (inPhase == iDataObjects->Phase)
			{
				iDataObjects->NextUpdateCycle = iDataObjects->NextUpdateCycle - 1;
				if ((iDataObjects->SpeedRef == 0) || (iDataObjects->NextUpdateCycle <= 0)) {
					iDataObjects->NextUpdateCycle = iDataObjects->SpeedRef;
					iDataObjects->NeedsUpdate = true;
				}
				
			}
		}
	}

	return 0;
}

/*********************************************************
	INTEGER DATAREFS
*********************************************************/

/* Universal Dataref GET Handler */
int UniversalDataRefGET_INT(void* inRefcon)
{
	int * my_var = (int *)inRefcon;
	return *my_var;
}

/* Universal Dataref SET Handler */
void UniversalDataRefSET_INT(void* inRefcon, int inValue)
{
	int * my_var = (int *)inRefcon;

	if (InternalDatarefUpdate == false) {
		DebugOut("========== DATAREF SET RECEIVED - Searching for the internal A320 Object");
		/*Here the A320 Internal Var gets updated in case that it was not an internal
		  Dataref-Update */
		list<DataObject>::iterator  iDataObjects;

		for (iDataObjects = DataObjects.begin(); iDataObjects != DataObjects.end(); ++iDataObjects) {
			if (iDataObjects->pAdress == inRefcon) {
				
				DebugOut("========== Found " + iDataObjects->DataRef);

				if (iDataObjects->FFID <= 0) {
					iDataObjects->FFID = ffAPI.ValueIdByName(iDataObjects->FFVar.c_str());
				}

				ffAPI.ValueSet(iDataObjects->FFID, &inValue);

				break;
			}
		}
		DebugOut("========== DATAREF SET Done.");

	}


	*my_var = inValue;
}

/*********************************************************
	FLOAT DATAREFS
*********************************************************/

/* Universal Dataref GET Handler */
float UniversalDataRefGET_FLOAT(void* inRefcon)
{
	float * my_var = (float *)inRefcon;
	return *my_var;
}

/* Universal Dataref SET Handler */
void UniversalDataRefSET_FLOAT(void* inRefcon, float inValue)
{
	float * my_var = (float *)inRefcon;

	if (InternalDatarefUpdate == false) {
		DebugOut("========== DATAREF SET RECEIVED - Searching for the internal A320 Object");
		/*Here the A320 Internal Var gets updated in case that it was not an internal
		Dataref-Update */
		list<DataObject>::iterator  iDataObjects;

		for (iDataObjects = DataObjects.begin(); iDataObjects != DataObjects.end(); ++iDataObjects) {
			if (iDataObjects->pAdressf == inRefcon) {

				DebugOut("========== Found " + iDataObjects->DataRef);

				if (iDataObjects->FFID <= 0) {
					iDataObjects->FFID = ffAPI.ValueIdByName(iDataObjects->FFVar.c_str());
				}

				ffAPI.ValueSet(iDataObjects->FFID, &inValue);

				break;
			}
		}
		DebugOut("========== DATAREF SET Done.");

	}


	*my_var = inValue;
}


/*
* DumpObjectsToLog
*
* Dumps all Objects and Parameters to the Log.txt
*
*/
void DumpObjectsToLog() {
	LogWrite("=============== DUMP OF ALL A320 OBJECTS AND PARAMETERS =================");
	unsigned int valuesCount = ffAPI.ValuesCount();
	int valueID = -1;

	unsigned int ii = 0;
	for (ii = 0; ii < valuesCount; ii++) {
		int TmpParentID = -1;
		int TmpValueID = -1;
		string FullObjectName = "";

		char *valueName, *valueDescription;

		valueID = ffAPI.ValueIdByIndex(ii);

		if (valueID >= 0) { 
			valueName = (char *)ffAPI.ValueName(valueID);
			valueDescription = (char *)ffAPI.ValueDesc(valueID);

			unsigned int valueType = ffAPI.ValueType(valueID);
			unsigned int valueFlag = ffAPI.ValueFlags(valueID);

			int parentValueID = ffAPI.ValueParent(valueID);

			// Here we get all the parents to get the full name of the object
			TmpValueID = valueID;
			TmpParentID = parentValueID;
			while ((TmpParentID > 0) && (TmpValueID > 0)) {
				TmpParentID = ffAPI.ValueParent(TmpValueID);
				if ((TmpParentID >= 0) && (TmpValueID >= 0)) FullObjectName = string((char *)ffAPI.ValueName(TmpParentID)) + string(".") + FullObjectName;
				TmpValueID = TmpParentID;
			}
			FullObjectName += string(valueName);

			char *valueTypeString;

			if (valueType == Value_Type_Deleted) {
				valueTypeString = "Deleted";
			}
			else if (valueType == Value_Type_Object) {
				valueTypeString = "Object";
			}
			else if (valueType == Value_Type_sint8) {
				valueTypeString = "sint8";
			}
			else if (valueType == Value_Type_uint8) {
				valueTypeString = "uint8";
			}
			else if (valueType == Value_Type_sint16) {
				valueTypeString = "sint16";
			}
			else if (valueType == Value_Type_uint16) {
				valueTypeString = "uint16";
			}
			else if (valueType == Value_Type_sint32) {
				valueTypeString = "sint32";
			}
			else if (valueType == Value_Type_uint32) {
				valueTypeString = "uint32";
			}
			else if (valueType == Value_Type_float32) {
				valueTypeString = "float32";
			}
			else if (valueType == Value_Type_float64) {
				valueTypeString = "float64";
			}
			else if (valueType == Value_Type_String) {
				valueTypeString = "String";
			}
			else if (valueType == Value_Type_Time) {
				valueTypeString = "Time";
			}
			else {
				valueTypeString = "UNKNOWN";
			}

			LogWrite("#" + to_string(valueID) + ": " + FullObjectName + " - " + string(valueDescription) + " (" + valueTypeString + ")" + " Value-Flag: " + to_string(valueFlag));

		}

	}
	LogWrite("=============== DUMP END =================");

	DumpObjectsToLogActive = false;
}


/*
* ReadIni
*
* Reads the ini from plugin or aircraft directory and initializes the DataObjects
*
*/
void ReadIni() {
	
	/* Leave if plugin was disabled */
	if (plugindisabled == true) return;

	/* First destroy and clear all Objects */
	list<DataObject>::iterator  iDataObjects;

	for (iDataObjects = DataObjects.begin(); iDataObjects != DataObjects.end(); ++iDataObjects) {
		iDataObjects->destroy();
	}
	DataObjects.clear();

	/* Where is our config file? */
	string filename;

	/* Getting the Aircraft Directory */
	char cacfilename[256] = { 0 };
	char cacpath[512] = { 0 };
	XPLMGetNthAircraftModel(0, cacfilename, cacpath);
	string aircraft_path = string(cacpath).substr(0, string(cacpath).find_last_of("\\/"));	// Remove the Filename from the complete path
	
	/* Check if the plugin is in the Aircraft Directory */
	if (file_exists(aircraft_path + string(XPLMGetDirectorySeparator()) + "plugins" + string(XPLMGetDirectorySeparator()) + "FFA320Connector" + string(XPLMGetDirectorySeparator()) + "config.cfg")) {
		filename = aircraft_path + string(XPLMGetDirectorySeparator()) + "plugins" + string(XPLMGetDirectorySeparator()) + "FFA320Connector" + string(XPLMGetDirectorySeparator()) + "config.cfg";
		LogWrite("Plugin is installed in the Aircraft-Directory. Reading Config from " + filename);
	}
	/* Or in the main plugin directory */
	else {
		/* Getting the Main plugins Directory */
		char PluginINIFile[512];
		XPLMGetSystemPath(PluginINIFile);
		strcat(PluginINIFile, "Resources");
		strcat(PluginINIFile, XPLMGetDirectorySeparator());
		strcat(PluginINIFile, "plugins");
		strcat(PluginINIFile, XPLMGetDirectorySeparator());
		strcat(PluginINIFile, "FFA320Connector");
		strcat(PluginINIFile, XPLMGetDirectorySeparator());
		strcat(PluginINIFile, "config.cfg");
		filename = string(PluginINIFile);
		LogWrite("Plugin is installed in the main Plugins-Directory. Reading Config from " + filename);
	}


	ifstream input(filename.c_str());

	if (!input)
	{
		LogWrite("==== ERROR: COULD NOT READ CONFIG ===");
		return;
	}

	string line;
	int objcounter = 0;
	int datarefcounter = 0;
	while (std::getline(input, line))
	{
		objcounter++;
		bool isCommand = false;
		bool isInt = false;
		int i = 0;

		if (line.substr(0, 1) != "#") {
			if (line.find(";") > 4) {
				string s = line;
				string delimiter = ";";

				size_t pos = 0;
				string token;		// Each token between ";"

				DataObject NewObj;	// Create a new Data Object for the Command / Dataref

				NewObj.DataRefMultiplier = 1;	// Sets the Multiplier to 1 - may be changed later on

				while ((pos = s.find(delimiter)) != std::string::npos) {
					token = s.substr(0, pos);

					if ((i == 0) && (token == "COMMAND"))	NewObj.Type = StringToObjectType("COMMAND");
					if ((i == 0) && (token == "DATAREF"))	NewObj.Type = StringToObjectType("DATAREF");

					//Command
					if (NewObj.Type == OBJECT_TYPE_COMMAND) {
						if (i == 1) NewObj.WorkMode = StringToWorkMode(token);
						if (i == 2) NewObj.ValueType = StringToValueType(token);
						if (i == 3) NewObj.FFVar = token;
						if (i == 4) NewObj.Command = token;
						if (i == 5) NewObj.CommandName = token;
						if (i == 6) {
							if (NewObj.ValueType == VALUE_TYPE_INT) NewObj.Value = stoi(token);
							if (NewObj.ValueType == VALUE_TYPE_FLOAT) NewObj.ValueFloat = stof(token);
						}
						if (i == 7) NewObj.FFID = stoi(token);
						if (i == 8) NewObj.FFReference = token;
						if (i == 9) NewObj.FFReferenceID = stoi(token);
						if (i == 10) {
							if (NewObj.ValueType == VALUE_TYPE_INT) NewObj.MinValue = stoi(token);
							if (NewObj.ValueType == VALUE_TYPE_FLOAT) NewObj.MinValueFloat = stof(token);
						}
						if (i == 11) {
							if (NewObj.ValueType == VALUE_TYPE_INT) NewObj.MaxValue = stoi(token);
							if (NewObj.ValueType == VALUE_TYPE_FLOAT) NewObj.MaxValueFloat = stof(token);
						}
						if (i == 12) NewObj.SpeedRef = stoi(token);
						if (i == 13) NewObj.Phase = stoi(token);

						NewObj.NeedsUpdate = false;
					}

					//Dataref
					if (NewObj.Type == OBJECT_TYPE_DATAREF) {
						if (i == 1) NewObj.ValueType = StringToValueType(token);
						if (i == 2) NewObj.FFVar = token;
						if (i == 3) NewObj.FFID = stoi(token);
						if (i == 4) NewObj.DataRef = token;
						if (i == 5) NewObj.DataRefValueType = StringToValueType(token);
						if ((i == 6) && (token == "IGNOREEXISTING")) NewObj.IgnoreExistingDataRef = true;
						if ((i == 7) && (token != "")) NewObj.DataRefMultiplier = stof(token); 

						if (NewObj.DataRef == "NORM") {
							string normdref = NewObj.FFVar;
							replace(normdref.begin(), normdref.end(), '.', '/');
							NewObj.DataRef = "MOKNY/FFA320/" + normdref;
						}

						NewObj.NeedsUpdate = true;
					}


					s.erase(0, pos + delimiter.length());
					i++;
				}

				if (i > 1) {
					NewObj.RefConID = datarefcounter++;
					NewObj.initialize();
					DataObjects.push_back(NewObj);
				}
			}
		}
	}
	
}

/*
* ffAPIUpdateCallback
*
* FlightFactor Callback. Here we process the DataObject thingy
*
*/
void ffAPIUpdateCallback(double step, void *tag) {

	/* Leave if plugin was disabled */
	if (plugindisabled == true) return;

	/* When this is called, we know that the plugin is initialized correctly */
	plugininitialized = true;

	if (DumpObjectsToLogActive == true) DumpObjectsToLog();

	/* Iterate thru the Objects and see what object needs to be updated */
	list<DataObject>::iterator  iDataObjects;

	for (iDataObjects = DataObjects.begin(); iDataObjects != DataObjects.end(); ++iDataObjects) {
		
		/* Resetting the Click-Commands to the MinValue */
		if (iDataObjects->NeedsClickUpdate == true) {
			if (iDataObjects->ClickTimer <= 0) {
				if (iDataObjects->ValueType == VALUE_TYPE_INT) ffAPI.ValueSet(iDataObjects->FFID, &iDataObjects->MinValue);
				if (iDataObjects->ValueType == VALUE_TYPE_FLOAT) ffAPI.ValueSet(iDataObjects->FFID, &iDataObjects->MinValueFloat);
				iDataObjects->NeedsClickUpdate = false;
			}
			else {
				iDataObjects->ClickTimer--;
			}
		}

		/* The main interface between XP and the A320 */
		if (iDataObjects->NeedsUpdate == true) {
				
			/* First find and set the Object-ID - This is only done once per Object (if needed) */
			if (iDataObjects->FFID <= 0) {
				iDataObjects->FFID = ffAPI.ValueIdByName(iDataObjects->FFVar.c_str());
			}

			/* First find and set the Object-Reference-ID - This is only done once per Object (if needed) */
			if (iDataObjects->FFReferenceID <= 0) {
				iDataObjects->FFReferenceID = ffAPI.ValueIdByName(iDataObjects->FFReference.c_str());
			}

			/* Executed Commands are ported to the A320 here */
			if (iDataObjects->Type == OBJECT_TYPE_COMMAND) {

				DebugOut("Updating COMMAND " + iDataObjects->Command + " - " + iDataObjects->CommandName);

				/* Workmode SET */
				if (iDataObjects->WorkMode == WORK_MODE_SET) {
					if (iDataObjects->ValueType == VALUE_TYPE_INT) ffAPI.ValueSet(iDataObjects->FFID, &iDataObjects->Value);
					if (iDataObjects->ValueType == VALUE_TYPE_FLOAT) ffAPI.ValueSet(iDataObjects->FFID, &iDataObjects->ValueFloat);
				}

				/* Workmode STEP */
				if (iDataObjects->WorkMode == WORK_MODE_STEP) {
					if (iDataObjects->ValueType == VALUE_TYPE_INT) {
						int curval;
						ffAPI.ValueGet(iDataObjects->FFReferenceID, &curval);

						if ((curval + iDataObjects->Value <= iDataObjects->MaxValue) && (curval + iDataObjects->Value >= iDataObjects->MinValue)) {
							int newval = curval + iDataObjects->Value;
							ffAPI.ValueSet(iDataObjects->FFID, &newval);
						}
					}
					if (iDataObjects->ValueType == VALUE_TYPE_FLOAT) {
						float curval;
						ffAPI.ValueGet(iDataObjects->FFReferenceID, &curval);

						if ((curval + iDataObjects->ValueFloat <= iDataObjects->MaxValueFloat) && (curval + iDataObjects->ValueFloat >= iDataObjects->MinValueFloat)) {
							float newval = curval + iDataObjects->ValueFloat;
							ffAPI.ValueSet(iDataObjects->FFID, &newval);
						}
					}
				}

				/* Workmode CYCLE */
				if (iDataObjects->WorkMode == WORK_MODE_CYCLE) {
					if (iDataObjects->ValueType == VALUE_TYPE_INT) {
						int curval;
						ffAPI.ValueGet(iDataObjects->FFReferenceID, &curval);

						int newval = curval + iDataObjects->Value;
						if (newval > iDataObjects->MaxValue) newval = iDataObjects->MinValue;
						if (newval < iDataObjects->MinValue) newval = iDataObjects->MaxValue;

						ffAPI.ValueSet(iDataObjects->FFID, &newval);
					}
					if (iDataObjects->ValueType == VALUE_TYPE_FLOAT) {
						float curval;
						ffAPI.ValueGet(iDataObjects->FFReferenceID, &curval);

						float newval = curval + iDataObjects->ValueFloat;
						if (newval > iDataObjects->MaxValueFloat) newval = iDataObjects->MinValueFloat;
						if (newval < iDataObjects->MinValueFloat) newval = iDataObjects->MaxValueFloat;

						ffAPI.ValueSet(iDataObjects->FFID, &newval);
					}
				}

				/* Workmode ROTATE */
				if (iDataObjects->WorkMode == WORK_MODE_ROTATE) {
					if (iDataObjects->ValueType == VALUE_TYPE_INT) {
						int curval;
						ffAPI.ValueGet(iDataObjects->FFReferenceID, &curval);
						int newval = curval + iDataObjects->Value;
						ffAPI.ValueSet(iDataObjects->FFID, &newval);
					}
					if (iDataObjects->ValueType == VALUE_TYPE_FLOAT) {
						float curval;
						ffAPI.ValueGet(iDataObjects->FFReferenceID, &curval);
						float newval = curval + iDataObjects->ValueFloat;
						ffAPI.ValueSet(iDataObjects->FFID, &newval);
					}
				}

				/* Workmode CLICK */
				if (iDataObjects->WorkMode == WORK_MODE_CLICK) {
					if (iDataObjects->ValueType == VALUE_TYPE_INT) {
						ffAPI.ValueSet(iDataObjects->FFID, &iDataObjects->Value);
						iDataObjects->NeedsClickUpdate = true;
						iDataObjects->ClickTimer = 5;
					}
					if (iDataObjects->ValueType == VALUE_TYPE_FLOAT) {
						ffAPI.ValueSet(iDataObjects->FFID, &iDataObjects->ValueFloat);
						iDataObjects->NeedsClickUpdate = true;
						iDataObjects->ClickTimer = 5;
					}
				}
				iDataObjects->NeedsUpdate = false;
			}

			/* Datarefs are hanlded here */
			if (iDataObjects->Type == OBJECT_TYPE_DATAREF) {
				DebugOut("Updating DATAREF " + iDataObjects->DataRef  + " - " + to_string(iDataObjects->ValueType) + " - " + iDataObjects->FFVar);
				
				InternalDatarefUpdate = true;	// While this is true, no external DataRef-Updates are triggered. Otherwise we can not decide if the update
												// comes from an external source or from this plugin.
	
				// Set the Dataref according to the Dataref Value Type INT / FLOAT
				if (iDataObjects->DataRefValueType == VALUE_TYPE_INT)  {
					DebugOut(" -> INT TO INT");
					void* curval;
					ffAPI.ValueGet(iDataObjects->FFID, &curval);
					XPLMSetDatai(iDataObjects->DREF, (int)curval * iDataObjects->DataRefMultiplier);
				}
				
				if (iDataObjects->DataRefValueType == VALUE_TYPE_FLOAT) {
					
					//Read the A320 Data in the specific format
					if (iDataObjects->ValueType == VALUE_TYPE_INT) {
						DebugOut(" -> INT TO FLOAT");
						int icurval;
						ffAPI.ValueGet(iDataObjects->FFID, &icurval);
						XPLMSetDataf(iDataObjects->DREF, (float)icurval * iDataObjects->DataRefMultiplier);
					} else if (iDataObjects->ValueType == VALUE_TYPE_FLOAT) {
						DebugOut(" -> FLOAT TO FLOAT");
						float fcurval;
						ffAPI.ValueGet(iDataObjects->FFID, &fcurval);
						XPLMSetDataf(iDataObjects->DREF, fcurval * iDataObjects->DataRefMultiplier);
					}
					
				}

				InternalDatarefUpdate = false;

			}

		}



	}

}

/*
* PluginCustomFlightLoopCallback
*
* This is called once for initialization
*
*/
float PluginCustomFlightLoopCallback(float elapsedMe, float elapsedSim, int counter, void * refcon)
{
	/* Export the Datarefs to the DRE */
	list<DataObject>::iterator  iDataObjects;
	XPLMPluginID PluginID = XPLMFindPluginBySignature("xplanesdk.examples.DataRefEditor");
	for (iDataObjects = DataObjects.begin(); iDataObjects != DataObjects.end(); ++iDataObjects) {
		if (iDataObjects->Type == OBJECT_TYPE_DATAREF) {
			if ((PluginID != XPLM_NO_PLUGIN_ID)){
				XPLMSendMessageToPlugin(PluginID, MSG_ADD_DATAREF, (void*)iDataObjects->DataRef.c_str());
			}
		}
	}

	/* Finding the FF A320 API */
	if (ffPluginID == XPLM_NO_PLUGIN_ID) {
		ffPluginID = XPLMFindPluginBySignature(XPLM_FF_SIGNATURE);
		return -1.0;
	}

	/* Initializing the FF A320 API */
	XPLMSendMessageToPlugin(ffPluginID, XPLM_FF_MSG_GET_SHARED_INTERFACE, &ffAPI);

	if (ffAPI.DataVersion != NULL) {
		ffAPIdataversion = ffAPI.DataVersion();
	}

	if (ffAPI.DataAddUpdate != NULL) {
		ffAPI.DataAddUpdate(&ffAPIUpdateCallback, tag);
		return 0;
	}

	return -1.0;
}








/**********************************************/

/*
* XPluginStop
*
* Our cleanup routine deallocates our window.
*
*/
PLUGIN_API void	XPluginStop(void)
{
	
	/*********
		I had to uncomment the following lines, otherwise the
		plugin would crash XP when exiting from the initial menu or
		from a different A/C than the A320.
	***********/

	/*try {
		ffAPI.DataDelUpdate(&ffAPIUpdateCallback, tag);
		ffPluginID = XPLM_NO_PLUGIN_ID;
	}
	catch (int e) {

	}*/



}


/*
* XPluginDisable
*
* We do not need to do anything when we are disabled, but we must provide the handler.
*
*/
PLUGIN_API void XPluginDisable(void)
{
	plugindisabled = true;
}

/*
* XPluginEnable.
*
* We don't do any enable-specific initialization, but we must return 1 to indicate
* that we may be enabled at this time.
*
*/
PLUGIN_API int XPluginEnable(void)
{
	plugindisabled = false;
	return 1;
}

/*
* XPluginReceiveMessage
*
* We don't have to do anything in our receive message handler, but we must provide one.
*
*/
PLUGIN_API void XPluginReceiveMessage(
	XPLMPluginID	inFromWho,
	int				inMessage,
	void *			inParam)
{
}