import os

env = Environment(
	ENV = os.environ,
)

SConscript('laks/build_rules')

env.SelectMCU('stm32l151rb')

env.Firmware('demo.elf', Glob('*.cpp'))
