import os

env = Environment(
	ENV = os.environ,
)

SConscript('laks/build_rules')

env.SelectMCU('stm32f051r8')

env.Firmware('demo.elf', Glob('*.cpp'))
