Install:
	Install the complete package using nero.rosinstall
	$rosinstall ~/ros nero.rosinstall

	Install pocketsphinx for speech recognition by following this tutorial:
	http://www.ros.org/wiki/pocketsphinx
	
	Place udevRules/roman.rules in /etc/udev/rules.d/

Troubleshooting:
	If you are getting this error while building:
	"error: ‘TINYXML_ELEMENT’ is not a member of ‘TiXmlElement’"

	Go to this page and install libtinyxml and libtinyxml-dev for your architecture:
	http://packages.ros.org/ros/ubuntu/pool/main/t/tinyxml/

	Adjust lines 24 and 27 from CDxlROSPacketHandler.cpp in package threemxl. The node needs to use persistent connections, do this by replacing the lines with
	"sendto_service_ = nh_.serviceClient<shared_serial::SendTo>("sendto", true);"
	
	and
	
	"recv_service_ = nh_.serviceClient<shared_serial::Recv>("recv", true);"
	
	respectively.
