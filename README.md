# Nuvoton-ASoC
Nuvoton codec and amp drivers with different ASoC versions.

The Nuvoton-ASoC repository stores the Linux driver for codec and amplifier manufactured by Nuvoton. In each chip folder, you can find drivers for different Linux versions. We will update these drivers when new patches are approved by the Kernel community.

x:none , o:ok (sync newest version)<br>
----------------------------------------------------------------------------------------<br>
Kernel      |k3.4-  k3.14  k3.18  k3.19  k4.4-  k4.18  k4.19  k5.10  |note<br>
            |k3.10                       k4.15                       |<br>
------------|--------------------------------------------------------|----------<br>
NAU8315     |x      x      x      x      x      x      x      x      |upstreame at k5.11<br>
NAU85L40    |x      x      x      x      x      x      x      o      |upstreame at 4.11<br>
NAU88C10    |x      x      x      x      x      x      x      o      |upstreame at 4.9<br>
NAU88L21    |x      x      x      x      x      x      x      x      |upstreame<br>
NAU88C22    |x      x      x      x      x      x      x      o      |upstreame at 4.20<br>
NAU88L24    |x      x      x      x      o      x      x      o      |upstreame at 4.12<br>
NAU88L25    |x      x      x      x      o      x      x      o      |upstreame at k4.4<br>
NAU83G10/20 |x      x      x      x      x      x      x      x      |<br>
<br>
k5.10: https://elixir.bootlin.com/linux/v5.10.70/source/sound/soc/codecs<br>
