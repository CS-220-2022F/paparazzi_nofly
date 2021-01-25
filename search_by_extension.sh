grep -n --color "$1" $(grep -lr "$1" $PAPARAZZI_HOME | grep "\\.$2$")
