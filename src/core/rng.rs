//! A small seeded random number generator.
//!
//! The simulator wants repeatable randomness (GNSS wander, gusts, radio
//! drops, oil spreading) and nothing cryptographic, so this is a
//! xoshiro256** with a Box–Muller normal on top.

#[derive(Clone, Debug)]
pub struct Rng {
    s: [u64; 4],
    spare: Option<f64>,
}

fn splitmix(state: &mut u64) -> u64 {
    *state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
    let mut z = *state;
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

impl Rng {
    pub fn new(seed: u64) -> Self {
        let mut state = seed;
        let s = [splitmix(&mut state), splitmix(&mut state), splitmix(&mut state), splitmix(&mut state)];
        Rng { s, spare: None }
    }

    pub fn next_u64(&mut self) -> u64 {
        let result = self.s[1].wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = self.s[1] << 17;
        self.s[2] ^= self.s[0];
        self.s[3] ^= self.s[1];
        self.s[1] ^= self.s[2];
        self.s[0] ^= self.s[3];
        self.s[2] ^= t;
        self.s[3] = self.s[3].rotate_left(45);
        result
    }

    /// Uniform in [0, 1).
    pub fn random(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    pub fn uniform(&mut self, low: f64, high: f64) -> f64 {
        low + (high - low) * self.random()
    }

    /// Normal with mean `mu` and standard deviation `sigma`.
    pub fn gauss(&mut self, mu: f64, sigma: f64) -> f64 {
        if let Some(z) = self.spare.take() {
            return mu + sigma * z;
        }
        loop {
            let u = 2.0 * self.random() - 1.0;
            let v = 2.0 * self.random() - 1.0;
            let s = u * u + v * v;
            if s > 0.0 && s < 1.0 {
                let k = (-2.0 * s.ln() / s).sqrt();
                self.spare = Some(v * k);
                return mu + sigma * u * k;
            }
        }
    }
}
