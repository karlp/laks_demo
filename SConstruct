import os

env = Environment(
	ENV = os.environ,
)

SConscript('laks/build_rules')

#env.SelectMCU('stm32f103cb')
env.SelectMCU('stm32f407zg')

env.Firmware('demo.elf', Glob('*.cpp'))
