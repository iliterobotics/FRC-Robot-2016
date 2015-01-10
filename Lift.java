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
			state = MotorState.STOP;
			toteCount++;
		}

		if(crateLiftLimitBottom.get()){
			if(state == MotorState.DOWN) {
				toteCount--;
			}
		} 
	}
}