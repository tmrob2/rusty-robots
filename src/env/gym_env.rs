use scpm::agent::Agent;

pub trait Env<T, S, W> where T: Agent<S, W>, W: Clone {
    fn make(na: i32, init_state: S) -> T;

    fn state_space(&mut self);

    fn step(&mut self, state: &S, a: i32) -> Result<Vec<(S, f64, W)>, &'static str>;

    fn transition_map(&mut self, r: &f64);
}

