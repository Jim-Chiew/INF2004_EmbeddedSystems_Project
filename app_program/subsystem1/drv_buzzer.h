/*
 *	drv_buzzer.h
 *	Subsystem 1 -- the piezo buzzer on GP22 (the Robo Pico's on-board one).
 *	Called ONLY by subsystem 1 (beeper.c), which owns the pin and PWM
 *	slice 3 (TEAM.md section 3).
 *
 *	A piezo needs a square wave, not a level: a passive one is silent if the
 *	pin is simply held high.  So the pin is driven by PWM at S1_BUZZER_HZ
 *	and "on" means 50 % duty.  (An active buzzer, which makes its own tone,
 *	sounds with this too, just a little quieter.)
 *
 *	These calls only start and stop the sound.  How long a beep lasts and
 *	how many there are is beeper.c's job -- nothing here ever waits.
 */

#ifndef DRV_BUZZER_H
#define DRV_BUZZER_H

void	buzzer_init(void);	/* pin + PWM, left silent */
void	buzzer_on(void);		/* start the usual tone (S1_BUZZER_HZ) */
void	buzzer_on_hz(unsigned hz);	/* ...or one of your own choosing */
void	buzzer_off(void);	/* silence */

#endif /* DRV_BUZZER_H */
