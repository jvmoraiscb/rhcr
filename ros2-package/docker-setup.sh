#!/bin/bash

sudo docker build -t rhcr-falcon .
cd
printf "#!/bin/bash\n\nsudo docker run -it --rm --network host --privileged -v /dev/bus/usb:/dev/bus/usb rhcr-falcon" > rhcr-falcon.sh
sudo chmod +x rhcr-falcon.sh