mod hidden;
mod triangle;

type Vector = nalgebra::Vector3<f32>;
type Point = nalgebra::Point3<f32>;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
