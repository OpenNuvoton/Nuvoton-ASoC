# Nuvoton-ASoC
Nuvoton codec and amp drivers with different ASoC versions.

The Nuvoton-ASoC repository stores the Linux driver for codec and amplifier manufactured by Nuvoton. In each chip folder, you can find drivers for different Linux versions. We will update these drivers when new patches are approved by the Kernel community.

x:none , o:ok (sync newest version)
|Kernel      |k3.4-10|k3.14  |k3.18  |k3.19  |k4.4-15|k4.18  |k4.19  |k5.10  |note|
|------------|-------|-------|-------|-------|-------|-------|-------|-------|----------|
|NAU8315     |x      |x      |x      |x      |x      |x      |x      |x      |upstreame at k5.11|
|NAU85L40    |x      |x      |x      |x      |x      |x      |x      |o      |upstreame at 4.11|
|NAU88C10    |o      |x      |o      |o      |o      |o      |o      |o      |upstreame at 4.9|
|NAU88L21    |x      |x      |o      |o      |o      |o      |o      |o      |upstreame|
|NAU88C22    |x      |x      |o      |o      |o      |o      |o      |o      |upstreame at 4.20|
|NAU88L24    |x      |x      |o      |o      |o      |o      |o      |o      |upstreame at 4.12|
|NAU88L25    |x      |x      |o      |o      |o      |o      |o      |o      |upstreame at k4.4|
|NAU83G10/20 |x      |x      |x      |x      |x      |x      |x      |x      ||

k5.10: https://elixir.bootlin.com/linux/v5.10.70/source/sound/soc/codecs
