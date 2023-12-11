#ifndef _SONAR_INFO_XML_H_
#define _SONAR_INFO_XML_H_
char sonar_info_xml[] = "\
<?xml version=\"1.0\" encoding=\"us-ascii\"?>\r\n\
<SB7125HF>\r\n\
  <Name deviceid=\"7125\" enumerator=\"0\" xml_reson=\"8.1.2.3.1.7\">7125 (400kHz)</Name>\r\n\
  <SystemInfo auv=\"no\" projectid=\"generic\" systemrelease=\"SV2\" name=\"7125\">7125 Sonar</SystemInfo>\r\n\
  <SonarType type=\"0\" unit=\"nb\">Bathymetric sonar</SonarType>\r\n\
  <ArrayType type=\"1\" number=\"7216\" unit=\"nb\">Flat array</ArrayType>\r\n\
  <RxElements min=\"0\" max=\"255\" spacing=\"0.0016\" unit=\"meters\" input_mux=\"1\">Receive ceramics</RxElements>\r\n\
  <RxBeamWidth uniformacross=\"yes\" uniformalong=\"yes\" across=\"0.008\" along=\"0.471238898\" unit=\"rad\">Receiver beamspacing</RxBeamWidth>\r\n\
  <RxBeamModeSet>\r\n\
    <RxBeamMode max_beams=\"256\" beam_spacing=\"EA\" min_coverage=\"45.0\" max_coverage=\"140.0\" min_grazing=\"15.0\" max_total_steering=\"85.0\">Minimum Beams</RxBeamMode>\r\n\
    <RxBeamMode max_beams=\"256\" beam_spacing=\"ID\" min_coverage=\"45.0\" max_coverage=\"140.0\" min_grazing=\"5.0\" max_total_steering=\"85.0\">Intermediate</RxBeamMode>\r\n\
    <RxBeamMode max_beams=\"256\" beam_spacing=\"ED\" min_coverage=\"45.0\" max_coverage=\"140.0\" min_grazing=\"15.0\" max_total_steering=\"85.0\">Best Coverage</RxBeamMode>\r\n\
  </RxBeamModeSet>\r\n\
  <RxBeamStabilization type=\"0\">Receiver beam stabilization</RxBeamStabilization>\r\n\
  <TxType type=\"2181-400\" number=\"2181\" unit=\"nb\">Standard</TxType>\r\n\
  <TxSetup P0=\"70\" TVR=\"disabled\" Droop=\"enabled\">Tx Additional Configuration</TxSetup>\r\n\
  <TxBeams min=\"0\" max=\"0\" unit=\"nb\">Transmit beams</TxBeams>\r\n\
  <TxBeamSteering steerable=\"no\" maxx=\"0.0\" minx=\"0.0\" maxz=\"0.0\" minz=\"0.0\" unit=\"rad\">Transmit beam steering</TxBeamSteering>\r\n\
  <TxBeamSpacing uniform=\"yes\" angles=\"0.0\" unit=\"rad\">Transmit beamspacing</TxBeamSpacing>\r\n\
  <TxBeamWidth variable=\"no\" maxx=\"0.0174533\" minx=\"0.0174533\" maxz=\"2.094395102\" minz=\"2.094395102\" unit=\"rad\">Transmit beamwidth</TxBeamWidth>\r\n\
  <TxBeamStabilization type=\"0\">Transmit beam stabilization</TxBeamStabilization>\r\n\
  <TxPulseLength min=\"0.000050\" max=\"0.000500\" min_FM=\"0.000300\" max_FM=\"0.020\" unit=\"s\" measured=\"1e-6\">Transmit pulse length</TxPulseLength>\r\n\
  <TxPulseEnvelope type=\"tukey\" parameter=\"0.1\"> TxPulseEnvelope </TxPulseEnvelope>\r\n\
  <TxDelay trigger=\"999e-6\" transmitter=\"999e-6\" units=\"sec\">Transmit Delay</TxDelay>\r\n\
  <Frequency chirp=\"no\" chirp_V6=\"yes\" min=\"360000.0\" max=\"440000.0\" center=\"400000.0\" unit=\"hz\">Transmit frequency</Frequency>\r\n\
  <DualHead MasterStart=\"380000\" MasterStop=\"400000\" SlaveStart=\"420000\" SlaveStop=\"400000\" unit=\"Hz\">Dual-Head FM frequencies</DualHead>\r\n\
  <FM_Parameters KaiserLevel_dB=\"3\" GuardBand_dB=\"3.0\" CorrelationIndex=\"0.5\" NegativeSweep=\"yes\">FM Tuning Paramters</FM_Parameters>\r\n\
  <SampleRate rate=\"78125.000\" unit=\"hz\">Receiver sample rate</SampleRate>\r\n\
  <Power min=\"200.0\" max=\"200.0\" tx_power_tweak=\"0.0\" shared=\"no\" unit=\"dB//uPa\">Transmit power</Power>\r\n\
  <Gain min=\"0.0\" max=\"83.0\" tvg_limit=\"83\" unit=\"dB\">Receiver gain</Gain>\r\n\
  <Range min=\"1.0\" max=\"100.0\" max_V6=\"100.0\" unit=\"m\">Operating range</Range>\r\n\
  <RangeSet size=\"12\" _1=\"5\" _2=\"8\" _3=\"10\" _4=\"15\" _5=\"20\" _6=\"25\" _7=\"30\" _8=\"35\" _9=\"40\" _10=\"50\" _11=\"75\" _12=\"100\"  unit=\"m\">Valid Range Set</RangeSet>\r\n\
  <PingRate min=\"0.0\" max=\"50.0\" ratio=\"1.0\" freerun=\"no\" unit=\"p/s\">Ping rate</PingRate>\r\n\
  <Motion rollable=\"no\" pitchable=\"no\" heavable=\"no\" roll=\"0\" pitch=\"0\" heave=\"0\" roll_ON=\"no\" pitch_ON=\"no\">Motion compensation factor</Motion>\r\n\
  <FWInfo type=\"single\" pps=\"new\" bite=\"new\" bf_upm_level=\"3\">Firmware Info</FWInfo>\r\n\
  <FWFiles bitfile_V6=\"v6_256_20120422.bit\" bitfile=\"bf256_20110808.bit\" BITEfile=\"7K_Bite_7125_SV2.xml\">Firmware Sonar Specific Files</FWFiles>\r\n\
  <FWFilterFiles_V6 size=\"4\" _1=\"0.0000999,filter1_v5_7125_7130_20110814.dat\" _2=\"0.000199,filter2_v5_7125_7130_20110814.dat\" _3=\"0.000299,filter3_v5_7125_7130_20110814.dat\" _4=\"999,filter4_v5_7125_7130_20110814.dat\">Firmware Filter Sonar Specific Files</FWFilterFiles_V6>\r\n\
  <FWFilterFiles size=\"4\" _1=\"0.0000999,bf_filter1_7125_20110705.dat\" _2=\"0.000199,bf_filter2_7125_20110705.dat\" _3=\"0.000299,bf_filter3_7125_20110705.dat\" _4=\"999,bf_filter4_7125_20110705.dat\">Firmware Filter Sonar Specific Files</FWFilterFiles>\r\n\
  <FPGA TxRxDelayOffset=\"0\" tx_skip=\"0\" rx_skip=\"0\" lo_if=\"600000.0\" delay=\"471e-6\" offset_size=\"4\" _1=\"4860\" _2=\"4860\" _3=\"4860\" _4=\"4860\">FPGA Sonar Specific Values</FPGA>\r\n\
  <DownLink register=\"yes\" remote=\"yes\">Downlink Commands</DownLink>\r\n\
  <RDR defaultpath=\"D:\\Data\\\" maxsize=\"1024000000\" units=\"Bytes\" format=\"short\" buffer=\"10\">Raw data recording</RDR>\r\n\
  <StartState maxpower=\"0.0\" ping=\"yes\" selected=\"yes\" udp=\"on\" calibrate=\"yes\" swiothrottlems=\"0\">Initial overwrite values</StartState>\r\n\
  <BottomDetection method=\"G2\">BD Method (G1_Simple, G1_BlendFilt, G2)</BottomDetection>\r\n\
  <Warnings PPS=\"fatal\" IOM=\"error\">Warning overrides</Warnings>\r\n\
  <GUIState wedgethrottlems=\"160\">Initial overwrite values</GUIState>\r\n\
  <AutoPilot FactoryDefaultTable=\"7125_v6_400kHz_Default.apc\">AutoPilot settings</AutoPilot>\r\n\
  <OutputBoost_dB Value=\"0\">Beamformer Output Boost</OutputBoost_dB>\r\n\
</SB7125HF>\r\n\
";
#endif
