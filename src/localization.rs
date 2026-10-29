use glam::{Vec3, Quat};

use crate::comms::Messagable;

/**
 * c++ vers
 * 
void LocalizationOutput::into_message(std::vector<uint8_t> &data) {
	float floats[7] = {
		// quat follows ordering of https://www.npmjs.com/package/quaternion
		S(orientation),
		X(orientation),
		Y(orientation),
		Z(orientation),
		X(position),
		Y(position),
		Z(position)
	};

	for (int i = 0; i < 7; i++) {
		uint32_t uint32_data = std::bit_cast<uint32_t>(floats[i]);

		data.push_back((uint8_t)(uint32_data >> 24));
		data.push_back((uint8_t)((uint32_data >> 16) & 0xFF));
		data.push_back((uint8_t)((uint32_data >> 8) & 0xFF));
		data.push_back((uint8_t)(uint32_data & 0xFF));
	}
}
 */

#[derive(Default, Clone, Copy, Debug)]
pub struct LocalizationOutput {
    position: Vec3,
    velocity: Vec3,
    ori: Quat
}

impl LocalizationOutput {
    pub fn new(position: Vec3, velocity: Vec3, ori: Quat) -> Self {
        Self {
            position,
            velocity,
            ori
        }
    }
}

impl Messagable for LocalizationOutput {
    fn insert_msg_bytes(&self, dst: &mut [u8]) -> usize {
        let floats: [f32; 7] = [
            self.ori.w,
            self.ori.x,
            self.ori.y,
            self.ori.z,
            // self.position.x,
            // self.position.y,
            // self.position.z,
            self.velocity.x,
            self.velocity.y,
            self.velocity.z
        ];

        for i in 0..floats.len() {
            let uint32_data = floats[i].to_bits();

            dst[4*i] = (uint32_data >> 24) as u8;
            dst[1 + 4*i] = ((uint32_data >> 16) & 0xFF) as u8;
            dst[2 + 4*i] = ((uint32_data >> 8) & 0xFF) as u8;
            dst[3 + 4*i] = (uint32_data & 0xFF) as u8;
        }
        28 // Return the number of bytes written
    }

    fn try_from_msg_bytes(_src: &[u8]) -> Option<Self> {
        None // TODO: Implement this if needed
    }
}