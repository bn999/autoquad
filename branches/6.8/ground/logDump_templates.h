
// GPX format templates
static const char gpxHeader[] = "<?xml version=\"1.0\"?>\n\
<gpx creator=\"AutoQuad logDump\" version=\"1.0\" xmlns=\"http://www.topografix.com/GPX/1/0\" \n\
	xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" xsi:schemaLocation=\"http://www.topografix.com/GPX/1/0 http://www.topografix.com/GPX/1/0/gpx.xsd\">\n";
static const char gpxFooter[] = "</gpx>\n";
static const char gpxTrkStart[] = "<trk>\n\
	<name>%s</name>\n\
	<trkseg>\n";
static const char gpxTrkEnd[] = "	</trkseg>\n</trk>\n";
static const char gpxTrksegDivider[] = "	</trkseg></trk>\n	<trk><trkseg>\n";
// str replace order: lat, lon, alt, time, heading, speed
static const char gpxTrkptTempl[] = "<trkpt lat=\"%.9f\" lon=\"%.9f\">\n\
	<ele>%.8f</ele>\n\
	<time>%s</time>\n\
	<course>%.3f</course>\n\
	<speed>%.3f</speed>\n\
</trkpt>\n";
// str replace order: lat, lon, alt, time, heading, speed, wpt name
static const char gpxWptTempl[] = "<wpt lat=\"%.9f\" lon=\"%.9f\">\n\
	<ele>%.8f</ele>\n\
	<time>%s</time>\n\
	<course>%.3f</course>\n\
	<speed>%.3f</speed>\n\
	<name>%s</name>\n\
</wpt>\n";

// KML format templates
// str replace order: document title, wpt color, wpt icon, wpt color, wpt icon, wpt trg color, wpt icon, wpt trg color, wpt icon, line color, line width (d), line color, line width (d)
static const char kmlHeader[] = "<?xml version=\"1.0\" encoding=\"ISO-8859-1\" standalone=\"yes\"?>\n\
<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\">\n\
	<Document>\n\
		<name><![CDATA[%s]]></name>\n\
		<description><![CDATA[&nbsp;]]></description>\n\
		<visibility>1</visibility>\n\
		<Style id=\"waypoint_normal\">\n\
			<IconStyle>\n\
				<color>%s</color><scale>0.4</scale>\n\
				<Icon><href>%s</href></Icon>\n\
				<hotSpot x=\"0.5\" xunits=\"fraction\" y=\"0.5\" yunits=\"fraction\" />\n\
			</IconStyle>\n\
			<LabelStyle><color>FFFFFFFF</color><scale>0</scale></LabelStyle>\n\
			<BalloonStyle><text><![CDATA[<p align=\"left\" style=\"white-space:nowrap;\"><font size=\"+1\"><b>$[name]</b></font></p> <p align=\"left\">$[description]</p>]]></text></BalloonStyle>\n\
		</Style>\n\
		<Style id=\"waypoint_highlight\">\n\
			<IconStyle>\n\
				<color>%s</color><scale>0.5</scale>\n\
				<Icon><href>%s</href></Icon>\n\
				<hotSpot x=\"0.5\" xunits=\"fraction\" y=\"0.5\" yunits=\"fraction\" />\n\
			</IconStyle>\n\
			<LabelStyle><color>FFFFFFFF</color><scale>0.5</scale></LabelStyle>\n\
			<BalloonStyle><text><![CDATA[<p align=\"left\" style=\"white-space:nowrap;\"><font size=\"+1\"><b>$[name]</b></font></p> <p align=\"left\">$[description]</p>]]></text></BalloonStyle>\n\
		</Style>\n\
		<StyleMap id=\"waypoint\">\n\
			<Pair><key>normal</key><styleUrl>#waypoint_normal</styleUrl></Pair>\n\
			<Pair><key>highlight</key><styleUrl>#waypoint_highlight</styleUrl></Pair>\n\
		</StyleMap>\n\
		<Style id=\"trg_waypoint_normal\">\n\
			<IconStyle>\n\
				<color>%s</color><scale>0.4</scale>\n\
				<Icon><href>%s</href></Icon>\n\
				<hotSpot x=\"0.5\" xunits=\"fraction\" y=\"0.5\" yunits=\"fraction\" />\n\
			</IconStyle>\n\
			<LabelStyle><color>FFFFFFFF</color><scale>0</scale></LabelStyle>\n\
			<BalloonStyle><text><![CDATA[<p align=\"left\" style=\"white-space:nowrap;\"><font size=\"+1\"><b>$[name]</b></font></p> <p align=\"left\">$[description]</p>]]></text></BalloonStyle>\n\
		</Style>\n\
		<Style id=\"trg_waypoint_highlight\">\n\
			<IconStyle>\n\
				<color>%s</color><scale>0.5</scale>\n\
				<Icon><href>%s</href></Icon>\n\
				<hotSpot x=\"0.5\" xunits=\"fraction\" y=\"0.5\" yunits=\"fraction\" />\n\
			</IconStyle>\n\
			<LabelStyle><color>FFFFFFFF</color><scale>0.5</scale></LabelStyle>\n\
			<BalloonStyle><text><![CDATA[<p align=\"left\" style=\"white-space:nowrap;\"><font size=\"+1\"><b>$[name]</b></font></p> <p align=\"left\">$[description]</p>]]></text></BalloonStyle>\n\
		</Style>\n\
		<StyleMap id=\"trg_waypoint\">\n\
			<Pair><key>normal</key><styleUrl>#trg_waypoint_normal</styleUrl></Pair>\n\
			<Pair><key>highlight</key><styleUrl>#trg_waypoint_highlight</styleUrl></Pair>\n\
		</StyleMap>\n\
		<Style id=\"track_n\">\n\
			<IconStyle>\n\
				<color>990000aa</color><scale>0</scale><Icon><href>http://earth.google.com/images/kml-icons/track-directional/track-0.png</href></Icon>\n\
			</IconStyle>\n\
			<LineStyle><color>%s</color><width>%d</width></LineStyle>\n\
			<PolyStyle><color>4b00ff00</color></PolyStyle>\n\
			<LabelStyle><scale>0</scale></LabelStyle>\n\
		</Style>\n\
		<Style id=\"track_h\">\n\
			<IconStyle>\n\
				<color>990000aa</color><scale>0</scale><Icon><href>http://earth.google.com/images/kml-icons/track-directional/track-0.png</href></Icon>\n\
			</IconStyle>\n\
			<LineStyle><color>%s</color><width>%d</width></LineStyle>\n\
			<PolyStyle><color>4b00ff00</color></PolyStyle>\n\
			<LabelStyle><scale>.8</scale></LabelStyle>\n\
		</Style>\n\
		<StyleMap id=\"track\">\n\
			<Pair><key>normal</key><styleUrl>#track_n</styleUrl></Pair>\n\
			<Pair><key>highlight</key><styleUrl>#track_h</styleUrl></Pair>\n\
		</StyleMap>\n\
		<Schema id=\"schema\">\n\
			<gx:SimpleArrayField name=\"speed\" type=\"float\">\n\
				<displayName>Horiz. Speed (m/s)</displayName>\n\
			</gx:SimpleArrayField>\n\
		</Schema>\n";
static const char kmlFooter[] = "	</Document>\n</kml>\n";

// str replace order: folder ID, folder name
static const char kmlFolderHeader[] = "<Folder id=\"%s\">\n\
	<name>%s</name>\n\
	<visibility>1</visibility>\n\
	<open>0</open>\n";

static const char kmlFolderFooter[] = "</Folder>";

// str replace order: track name, track ID, alt. mode
static const char kmlTrkHeader[] = "<Placemark>\n\
	<name><![CDATA[%s]]></name>\n\
	<description><![CDATA[&nbsp;]]></description>\n\
	<styleUrl>#track</styleUrl>\n\
	<gx:MultiTrack id=\"%s\">\n\
		<gx:interpolate>0</gx:interpolate>\n";

static const char kmlTrkFooter[] = "	</gx:MultiTrack>\n</Placemark>\n";

// str replace order: track ID
static const char kmlTrkStart[] = "<gx:Track id=\"%s\">\n<altitudeMode>%s</altitudeMode>\n";
static const char kmlTrkEnd[] = "</gx:Track>\n";

// str replace order: track model URL
static const char kmlModel[] = "<Model><Orientation><heading>0</heading></Orientation><Link><href>%s</href></Link></Model>\n";

static const char kmlTrkTimestamp[] = "<when>%s</when>\n";
// str replace order: lon, lat, alt
static const char kmlTrkCoords[] = "<gx:coord>%.9f %.9f %.6f</gx:coord>\n";
// str replace order: heading, tilt, roll
static const char kmlTrkAngles[] = "<gx:angles>%.3f %.6f %.6f</gx:angles>\n";

static const char kmlTrkExtDataStart[] = "<ExtendedData><SchemaData schemaUrl=\"#schema\"><gx:SimpleArrayData name=\"speed\">\n";
static const char kmlTrkExtDataEnd[] = "</gx:SimpleArrayData></SchemaData></ExtendedData>\n";
// str replace order: speed
static const char kmlTrkExtDataValue[] = "<gx:value>%.4f</gx:value>\n";

// str replace order: wpt name, date/time, lat, lon, alt, speed, heading, roll, pitch, climb rate, isotime, wpt style, alt. mode, lon, lat, alt
static const char kmlWptTempl[] = "<Placemark>\n\
	<name><![CDATA[%s]]></name>\n\
	<description><![CDATA[\n\
		<i>Time:</i> %s<br/>\n\
		<i>Latitude:</i> %.9f&#176;<br/>\n\
		<i>Longitude:</i> %.9f&#176;<br/>\n\
		<i>Elevation:</i> %.3f m<br/>\n\
		<i>Speed:</i> %.5f m/s (%.3f km/h)<br/>\n\
		<i>Heading:</i> %.2f&#176;<br/>\n\
		<i>Roll:</i> %.2f&#176;<br/>\n\
		<i>Pitch:</i> %.2f&#176;<br/>\n\
		<i>Climb Rate (approx.):</i> %.5f m/s\n\
	]]></description>\n\
	<TimeStamp><when>%s</when></TimeStamp>\n\
	<styleUrl>#%s</styleUrl>\n\
	<Point>\n\
		<altitudeMode>%s</altitudeMode>\n\
		<coordinates>%.9f,%.9f,%.6f</coordinates>\n\
	</Point>\n\
</Placemark>\n";
