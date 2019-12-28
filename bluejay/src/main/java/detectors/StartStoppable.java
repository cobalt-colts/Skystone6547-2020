package detectors;

public abstract class StartStoppable {
	Thread loop = new Thread(this::loop);
	volatile boolean activated = false;

	public final void start() {
		activated = true;
		loop.start();
	}

	public final void stop() {
		//yes stop() is deprecated but there is no big penalty for premature death
		//and suspend is useless as well because each cycle is independent
		activated = false;
		loop.stop();
	}

	public abstract void loop();
	public abstract void begin();
	public abstract void end();

}
