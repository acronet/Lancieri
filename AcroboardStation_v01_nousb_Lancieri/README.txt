1. Copiare la cartella e rinominarla AcroboardStation_v01_nousb_Project
2. Aprire .\AcroboardStation_v01_nousb.atsln e sostituire "simple_logger" con "project"
3. Rinominare la cartella .\simple_logger in .\project
4. Rinominare .\project\simple_logger.c e .\project\simple_logger.proj in .\project\project.c e .\project\project.proj
5. Aprire .\project\project.proj e sostituire "simple_logger" con "project"
6. Aprire .\AcroboardStation_v01\drivers\sim900\sim900.c e sostituire "simple_logger" con "project" alla riga 16
7. Definire in .\AcroboardStation_v01\src\config\conf_board.h i pin utilizzati dai sensori e la versione della scheda
8. Cercare in ./project/services/datalogger/datalogger.c il testo "// SENSORCODE" e inserire il codice necessario
