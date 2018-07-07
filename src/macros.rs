#[macro_export]
macro_rules! map {
    [ $($key:expr => $value:expr),* $(,)? ] => {
        {
            let mut hash_map = FnvHashMap::default();

            $(hash_map.insert($key, $value);)*

            hash_map
        }
    }
}
