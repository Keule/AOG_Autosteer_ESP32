// WIFI handling 7. Maerz 2021 for ESP32  -------------------------------------------

void WiFi_handle_connection(void* pvParameters) {
    if (Set.DataTransVia > 10) { vTaskDelay(5000); } //start Ethernet first, if needed for data transfer
    for (;;) {
        if (WiFi_connect_step == 0) {
            if (Set.debugmode) { Serial.println("closing WiFi connection task"); }
            vTaskDelete(NULL);
        }
        else {
            vTaskDelay(500); // every half second
            now = millis();

            IPAddress gwIP, myIP, myAPorSTA;

            if (Set.debugmode) { 
                Serial.print("WiFi_connect_step: "); 
                Serial.println(WiFi_connect_step);
            }
            switch (WiFi_connect_step) {

                // WiFi network scan
            case 10:
                WiFi_netw_nr = 0;
                WebIORunning = false;
                WiFiUDPRunning = false;
                if (WiFi_network_search_timeout == 0) {
                    WiFi_network_search_timeout = now + (Set.timeoutRouter * 1000);
                }
#if HardwarePlatform == 1  // Nano33IoT
                WiFi.status();
                delay(1);
#endif
                WiFi_scan_networks();
                if (now > WiFi_network_search_timeout) {
                    WiFi_connect_step = 50;
                } else if (WiFi_netw_nr > 0) {
                    WiFi_connect_step++;
                    WiFi_network_search_timeout = 0;
                }
                break;

                // start WiFi station
            case 11:
                WiFi.mode(WIFI_STA);
                WiFi_connect_step++;
                break;

            case 12:
                if (WiFi_network_search_timeout == 0) {
                    WiFi_network_search_timeout = now + (Set.timeoutRouter * 500);
                }
                WiFi_STA_connect_network();
                WiFi_connect_step++;
                break;

            case 13:
                if (WiFi.status() != WL_CONNECTED) {
                    Serial.print(".");
                    if (now > WiFi_network_search_timeout) {
                        WiFi_STA_connect_call_nr++;
                        WiFi_connect_step = 17;
                        WiFi_network_search_timeout += (Set.timeoutRouter * 500);
                    }
                } else {
                    WiFi_connect_step++;
                    WiFi_network_search_timeout = 0;
                }
                break;

                // change IP / DHCP
            case 14:
                Serial.println();
                Serial.println("WiFi Client successfully connected");
                Serial.print("Connected IP - Address : ");
                myIP = WiFi.localIP();
                Serial.println(myIP);

                // set fixed last octet
                myIP[3] = Set.WiFi_myip[3];
                Serial.print("changing IP to: ");
                Serial.println(myIP);

                gwIP = WiFi.gatewayIP();
                if (!WiFi.config(myIP, gwIP, Set.mask, gwIP)) {
                    Serial.println("STA Failed to configure");
                }
                WiFi_connect_step++;
                break;

            case 15:
                myIP = WiFi.localIP();
                Serial.print("Connected IP - Address : ");
                Serial.println(myIP);
                WiFi_ipDestination = myIP;
                WiFi_ipDestination[3] = Set.WiFi_ipDest_ending;
                Serial.print("sending to IP - Address : ");
                Serial.println(WiFi_ipDestination);
                gwIP = WiFi.gatewayIP();
                Serial.print("Gateway IP - Address : ");
                Serial.println(gwIP);
                my_WiFi_Mode = 1; // WIFI_STA
                WiFi_connect_step = 20;
                break;

            case 17:
                if (WiFi_STA_connect_call_nr > 2) {
                    WiFi_connect_step = 50;
                    WiFi_netw_nr = 0;
                } else {
                    WiFi.disconnect();
                    vTaskDelay(2);
                    WiFi_connect_step++;
                    Serial.print("-");
                }
                break;

            case 18:
                WiFi.mode(WIFI_OFF);
                vTaskDelay(2);
                WiFi_connect_step = 11;
                break;

                // UDP setup
            case 20:
                if (WiFiUDPToAOG.begin(Set.PortAutostToAOG)) {
                    Serial.print("UDP writing to IP: "); Serial.println(WiFi_ipDestination);
                    Serial.print("UDP writing to port: "); Serial.println(Set.PortDestination);
                    Serial.print("UDP writing from port: "); Serial.println(Set.PortAutostToAOG);
                } else {
                    Serial.println("Error starting UDP");
                }
                WiFi_connect_step++;
                break;

            case 21:
                if (WiFiUDPFromAOG.begin(Set.PortFromAOG)) {
                    Serial.print("WiFi UDP Listening for AOG data to port: ");
                    Serial.println(Set.PortFromAOG);
                    Serial.println();
                    WiFiUDPRunning = true;
                } else {
                    Serial.println("Error starting UDP");
                }
                delay(2);
                WiFi_connect_step = 100;
                break;

                // start access point if no STA
            case 50:
                WiFi_Start_AP();
                WiFi_connect_step++;
                break;

            case 51:
                if (my_WiFi_Mode == 2) {
                    WiFi_connect_step++;
                }
                break;

            case 52:
                WiFiUDPToAOG.begin(Set.PortAutostToAOG);
                Serial.print("UDP writing to IP: "); Serial.println(WiFi_ipDestination);
                Serial.print("UDP writing to port: "); Serial.println(Set.PortDestination);
                Serial.print("UDP writing from port: "); Serial.println(Set.PortAutostToAOG);
                WiFi_connect_step++;
                break;

            case 53:
                WiFiUDPFromAOG.begin(Set.PortFromAOG);
                Serial.print("NTRIP WiFi UDP Listening to port: ");
                Serial.println(Set.PortFromAOG);
                Serial.println();
                delay(2);
                WiFi_connect_step = 100;
                break;

                // web interface
            case 100:
                WiFiStartServer();
                WiFi_connect_step++;
                break;

            case 101:
                WebIOTimeOut = millis() + (long(Set.timeoutWebIO) * 60000);
                xTaskCreate(doWebinterface, "WebIOHandle", 5000, NULL, 1, &taskHandle_WebIO);
                delay(300);
                WiFi_connect_step = 0;
                LED_WIFI_ON = true;
                Serial.println(); Serial.println();
                myAPorSTA = (WiFi_netw_nr == 0) ? WiFi.softAPIP() : WiFi.localIP();
                Serial.print("started settings Webinterface at: ");
                Serial.println(myAPorSTA);
                Serial.println("type IP in Internet browser to get to webinterface");
                Serial.print("you need to be in WiFi network ");
                Serial.println((WiFi_netw_nr == 0) ? Set.ssid_ap : Set.ssid1);
                Serial.println(); Serial.println();
#if useLED_BUILTIN
                digitalWrite(LED_BUILTIN, HIGH);
#endif
                digitalWrite(Set.LEDWiFi_PIN, Set.LEDWiFi_ON_Level);
                break;

            default:
                WiFi_connect_step++;
                Serial.print("default called at WiFi_connection_step ");
                Serial.println(WiFi_connect_step);
                break;
            }
        }
    }
}

//---------------------------------------------------------------------
// scanning for known WiFi networks

void WiFi_scan_networks() {
    Serial.println("scanning for WiFi networks");
    int n = WiFi.scanNetworks();
    Serial.print("scan done: ");
    if (n == 0) {
        Serial.println("no networks found");
    } else {
        Serial.print(n); Serial.println(" network(s) found");
        for (int i = 0; i < n; ++i) {
            Serial.println(String("#") + (i + 1) + " : " + WiFi.SSID(i));
        }
        delay(800);
        delay(500);
        for (int i = 0; i < n; ++i) {
            if (WiFi.SSID(i) == Set.ssid1) {
                Serial.println("Connecting to: " + WiFi.SSID(i));
                WiFi_netw_nr = 1;
                break;
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------
// connects to WiFi network

void WiFi_STA_connect_network() {
    WiFi.begin(Set.ssid1, Set.password1);
    // DHCP is default; no WiFi.config() call needed on first run
    delay(2);
}

//-------------------------------------------------------------------------------------------------
// start WiFi Access Point = only if no existing WiFi or connection fails

void WiFi_Start_AP() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(Set.ssid_ap, "");
    // wait until AP has a valid IP
    while (WiFi.softAPIP() == IPAddress(0,0,0,0)) {
        delay(100);
        Serial.print(".");
    }
    delay(150); // settle time

    WiFi.softAPConfig(Set.WiFi_gwip, Set.WiFi_gwip, Set.mask);
    delay(300);

    IPAddress myIP = WiFi.softAPIP();
    Serial.print("Accesspoint started - Name : ");
    Serial.println(Set.ssid_ap);
    Serial.print(" IP address: ");
    Serial.println(myIP);

    WiFi_ipDestination = myIP;
    WiFi_ipDestination[3] = 255;
    my_WiFi_Mode = WIFI_AP;
}


//=================================================================================================
//Ethernet handling for ESP32 14. Feb 2021
//-------------------------------------------------------------------------------------------------
void Eth_handle_connection(void* pvParameters) {
    unsigned long Eth_connect_timer = 0, now = 0;
    if (Set.timeoutRouter < 12) { if (WiFi_connect_step != 0) { vTaskDelay(18000); } }//waiting for WiFi to start first
    Serial.println("started new task: Ethernet handle connection");
    for (;;) { // MAIN LOOP
        now = millis();
        if (Set.debugmode) { Serial.print("Ethernet connection step: "); Serial.println(Eth_connect_step); }
        if (Eth_connect_step > 0) {
            if (now > Eth_connect_timer + 300) {
                switch (Eth_connect_step) {
                case 10:
                    Ethernet.init(Set.Eth_CS_PIN);
                    Eth_connect_step++;
                    break;
                case 11:
                    if (Set.Eth_static_IP) { Ethernet.begin(Set.Eth_mac, Set.Eth_myip); }
                    else {
                        Ethernet.begin(Set.Eth_mac); //use DHCP
                        if (Set.debugmode) { Serial.println("waiting for DHCP IP adress"); }
                    }
                    Eth_connect_step++;
                    break;
                case 12:
                    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
                        Serial.println("no Ethernet hardware, Data Transfer set to WiFi");
                        Eth_connect_step = 255;//no Ethernet, end Ethernet
                        if (Set.DataTransVia == 10) {
                            Set.DataTransVia = 7; //change DataTransfer to WiFi
                            if (EthDataTaskRunning) { vTaskDelete(taskHandle_DataFromAOGEth); delay(5); EthDataTaskRunning = false; }
                            if (!WiFiDataTaskRunning) {                               
                                xTaskCreate(getDataFromAOGWiFi, "DataFromAOGHandleWiFi", 5000, NULL, 1, &taskHandle_DataFromAOGWiFi);
                                delay(500);
                            }//start WiFi if not running
                        }
                    }
                    else {
                        Serial.println("Ethernet hardware found, checking for connection");
                        Eth_connect_step++;
                    }
                    break;
                case 13:
                    if (Ethernet.linkStatus() == LinkOFF) {
                        Serial.println("Ethernet cable is not connected. Retrying in 5 Sek.");
                        vTaskDelay(5000);
                    }
                    else { Serial.println("Ethernet status OK"); Eth_connect_step++; }
                    break;
                case 14:
                    Serial.print("Got IP ");
                    Serial.println(Ethernet.localIP());
                    if ((Ethernet.localIP()[0] == 0) && (Ethernet.localIP()[1] == 0) && (Ethernet.localIP()[2] == 0) && (Ethernet.localIP()[3] == 0)) {
                        //got IP 0.0.0.0 = no DCHP so use static IP
                        Set.Eth_static_IP = true;
                    }
                    //use DHCP but change IP ending (x.x.x.80)
                    if (!Set.Eth_static_IP) {
                        for (byte n = 0; n < 3; n++) {
                            Set.Eth_myip[n] = Ethernet.localIP()[n];
                            Eth_ipDestination[n] = Ethernet.localIP()[n];
                        }
                        Eth_ipDestination[3] = 255;
                        Ethernet.setLocalIP(Set.Eth_myip);
                    }
                    else {//use static IP
                        for (byte n = 0; n < 3; n++) {
                            Eth_ipDestination[n] = Set.Eth_myip[n];
                        }
                        Eth_ipDestination[3] = Set.Eth_ipDest_ending;
                        Ethernet.setLocalIP(Set.Eth_myip);
                    }
                    Eth_connect_step++;
                    break;
                case 15:
                    Serial.print("Ethernet IP of autosteer module: "); Serial.println(Ethernet.localIP());
                    Serial.print("Ethernet sending to IP: "); Serial.println(Eth_ipDestination);
                    //init UPD Port sending to AOG
                    if (EthUDPToAOG.begin(Set.PortAutostToAOG))
                    {
                        Serial.print("Ethernet UDP sending from port: ");
                        Serial.println(Set.PortAutostToAOG);
                    }
                    Eth_connect_step++;
                    break;
                case 16:
                    //init UPD Port getting Data from AOG
                    if (EthUDPFromAOG.begin(Set.PortFromAOG))
                    {
                        Serial.print("Ethernet UDP listening to port: ");
                        Serial.println(Set.PortFromAOG);
                    }
                    EthUDPRunning = true;
                    Eth_connect_step = 0;//done
                    break;

                default:
                    Eth_connect_step++;
                    break;
                }//switch
                Eth_connect_timer = millis();
            }
        }
        if ((Eth_connect_step > 240) || (Eth_connect_step == 0)) {
            Serial.println("closing Ethernet connection task");
            delay(1);
            vTaskDelete(NULL);
            delay(1);
        }
        vTaskDelay(320);
    }
}

