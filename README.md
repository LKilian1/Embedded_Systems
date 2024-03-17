# STM32 Positionsbestimmung mit MEM - Sensoren
### Projektübersicht
Im Mittelpunkt dieses Projekts steht die Entwicklung eines Systems zur relativen Positionsbestimmung mit MEMS-Sensoren (Micro-Electro-Mechanical Systems). Durch die Integration von Beschleunigungs- und Magnetfeldmessungen sollen physikalische Parameter mit moderner Technologie überwacht und analysiert werden.

### Struktur
Es gibt zwei Ordner im Repository.
Im Ordner core befindet sich die Funktion main.c für dieses Projekt.
Der Ordner drivers enthält die Bibliotheken für die beiden Sensoren.
Das Projekt wurde mit Hilfe der Software von STMicroelectronics erstellt.
Die Handhabung des Codes kann anhand des referenzierten Dokuments nachvollzogen werden.

### Komponenten
STM32 Nucleo-144 Entwicklungsboard: Das Herzstück des Projekts, das eine robuste Plattform für die Verarbeitung von Sensordaten in Echtzeit bietet.
Magnetometer (LIS2MDL) und Beschleunigungsmesser (LIS2DW12): Unverzichtbare Sensoren für die Erfassung von magnetischen Orientierungen und Beschleunigungswerten.
I2C-Kommunikation: Gewährleistet eine effiziente Datenübertragung zwischen dem STM32-Board und den Sensoren.
Einrichtung und Konfiguration
Hardware-Einrichtung
Folgen Sie den Schaltplänen im Dokument ES_leo_kilian.pdf, um die Sensoren an das STM32 Nucleo-144 Board anzuschließen.

### Software Anforderungen
STM32CubeIDE und STM32CubeMX: Für die Projektkonfiguration und -entwicklung.
MEMS-Software-Pakete: Enthält Bibliotheken für die Sensorintegration.
Implementierung
Datenerfassung
Sensordaten werden über I2C erfasst, verarbeitet und zur Bestimmung der relativen Position des Geräts verwendet.

### Positionsbestimmung
Kombiniert Sensordaten mit Hilfe eines Kalman-Filter-Algorithmus für eine genaue Positionsschätzung.Kombiniert Sensordaten mit Hilfe eines Kalman-Filter-Algorithmus für eine genaue Positionsschätzung.

### Herausforderungen und Lösungen
Die Herausforderungen der Software- und Hardware-Integration wurden durch detaillierte Konfiguration und Tests bewältigt.
Ein Kalman-Filter-Ansatz wurde gewählt, um Ungenauigkeiten der Sensoren zu kompensieren.
Zukünftige Richtungen
Verbesserungen werden sich auf die Verbesserung der Positionsgenauigkeit durch zusätzliche Sensorintegration und Algorithmusoptimierung konzentrieren.

### Appendix
Enthält detaillierte Informationen über die Einrichtung der Workstation, Schnittstellenfunktionen, Sensorkonfigurationsregister und eine Low-Level-Funktion zur Registerkonfiguration.

### Referenz 
Eine ausführliche Literaturübersicht und Anhänge finden Sie in ES_Leo_kilian.pdf.



