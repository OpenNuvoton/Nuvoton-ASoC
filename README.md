# Nuvoton-ASoC
Nuvoton codec and amp drivers with different ASoC versions.

The Nuvoton-ASoC repository stores the Linux driver for codec and amplifier manufactured by Nuvoton. In each chip folder, you can find drivers for different Linux versions. We will update these drivers when new patches are approved by the Kernel community.

x:none , o:ok (sync newest version)
|Kernel                        |k3.14  |k3.18  |k3.19  |k4.4-15|k4.18  |k4.19  |k5.4-10|k5.19  |k6.1   |k6.6   |note            |
|------------------------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|----------------|
|NAU8315/NAU8318               |o      |o      |o      |o      |o      |o      |o      |o      |x      |x      |upstream at 5.11|
|NAU8325                       |x      |x      |x      |x      |x      |x      |o      |x      |x      |o      |un-upstream     |
|NAU8421                       |x      |x      |x      |x      |x      |x      |x      |x      |o      |x      |un-upstream     |
|NAU85L40                      |o      |o      |o      |o      |o      |o      |o      |o      |x      |x      |upstream at 4.11|
|NAU88C10/NAU88C12/NAU88C14    |o      |o      |o      |o      |o      |o      |o      |o      |o      |o      |upstream at 4.9 |
|NAU88L11                      |o      |o      |o      |o      |o      |o      |o      |o      |x      |x      |un-upstream     |
|NAU88L21                      |o      |o      |o      |o      |o      |o      |o      |o      |x      |x      |upstream at 5.16|
|NAU88C22                      |o      |o      |o      |o      |o      |o      |o      |o      |x      |o      |upstream at 4.20|
|NAU88L24                      |o      |o      |o      |o      |o      |o      |o      |o      |x      |x      |upstream at 4.12|
|NAU88L25                      |o      |o      |o      |o      |o      |o      |o      |o      |x      |o      |upstream at 4.4 |
|NAU83G10/NAU83G20             |o      |o      |o      |o      |o      |o      |o      |o      |x      |x      |un-upstream     |
|NAU83G60                      |x      |x      |x      |x      |x      |x      |x      |x      |o      |x      |un-upstream     |
|NAU83110                      |x      |x      |x      |x      |x      |x      |x      |x      |x      |o      |un-upstream     |
k5.10: https://elixir.bootlin.com/linux/v5.10.70/source/sound/soc/codecs
