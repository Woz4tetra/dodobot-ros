ACTIVE=$1
if [ -z "$ACTIVE" ]
then
      echo "Set active true"
      ACTIVE=true
else
      echo "Set active false"
      ACTIVE=false
fi

rosservice call /dodobot/set_state "{active: $ACTIVE, reporting: true}"
