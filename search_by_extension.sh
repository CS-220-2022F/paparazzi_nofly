grep -n --with-filename --color "$1" $(grep -lr "$1" $PAPARAZZI_HOME | grep "\\.$2$")
