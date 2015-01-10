public class Lift {
	private MotorState state;
	private int toteCount;

	public Lift(MotorState state) {
		this.state = state;
		toteCount = 0;
	}

	public MotorState getMotorState() {
		return state;
	}

	public void checkLimit() {
		if(crateLiftLimitTop.get()) {	
			interstate = MotorState.STOP;
		}

		if(crateLiftLimitBottom.get()){
			if(interstate == MotorState.UP) {
				toteCount++;
			}
			else {
				toteCount--;
			}
		} 
	}
}