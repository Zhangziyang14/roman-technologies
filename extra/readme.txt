Install:
	Install the complete package using nero.rosinstall
	$rosinstall ~/ros nero.rosinstall

	Place udevRules/roman.rules in /etc/udev/rules.d/

Troubleshooting:
	If you are getting this error while building:
	"error: ‘TINYXML_ELEMENT’ is not a member of ‘TiXmlElement’"

	Go to this page and install libtinyxml and libtinyxml-dev for your architecture:
	http://packages.ros.org/ros/ubuntu/pool/main/t/tinyxml/
