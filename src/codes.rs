//! Reading OpenLR codes from parquet.

use std::fs::File;
use std::path::Path;

use anyhow::{anyhow, Context, Result};
use arrow::array::{Array, StringArray};
use arrow::datatypes::DataType;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

/// Read base64 OpenLR codes from the `openlr` column (any string type) of a
/// parquet file, skipping nulls. Dictionary-encoded strings work too. Errors if
/// the column is missing or not a string.
pub fn load_openlr_codes(path: &Path) -> Result<Vec<String>> {
    let file = File::open(path).context("Failed to open parquet file")?;
    let builder = ParquetRecordBatchReaderBuilder::try_new(file)?;

    // Check the schema, not each batch, so a file with no rows errors too.
    let field = builder
        .schema()
        .field_with_name("openlr")
        .map_err(|_| anyhow!("Missing column 'openlr'"))?;
    let is_string =
        |t: &DataType| matches!(t, DataType::Utf8 | DataType::LargeUtf8 | DataType::Utf8View);
    let ok = match field.data_type() {
        DataType::Dictionary(_, values) => is_string(values),
        t => is_string(t),
    };
    if !ok {
        return Err(anyhow!(
            "Column 'openlr' has type {:?}, expected a string type",
            field.data_type()
        ));
    }

    let mut codes = Vec::new();
    for batch_result in builder.build()? {
        let batch = batch_result?;
        let col = batch.column_by_name("openlr").unwrap();
        let col = arrow::compute::cast(col, &DataType::Utf8)?;
        let col = col.as_any().downcast_ref::<StringArray>().unwrap();
        codes.extend(col.iter().flatten().map(str::to_string));
    }

    Ok(codes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use arrow::array::{ArrayRef, Int64Array, LargeStringArray};
    use arrow::record_batch::RecordBatch;
    use std::sync::Arc;

    #[test]
    fn test_load_openlr_codes_string_types() {
        use arrow::array::StringViewArray;
        use arrow::datatypes::{Field, Schema};
        use parquet::arrow::ArrowWriter;

        let path =
            std::env::temp_dir().join(format!("openlr_codes_{}.parquet", std::process::id()));
        let write = |col: ArrayRef| {
            let schema = Schema::new(vec![Field::new("openlr", col.data_type().clone(), true)]);
            let batch = RecordBatch::try_new(Arc::new(schema), vec![col]).unwrap();
            let mut writer =
                ArrowWriter::try_new(File::create(&path).unwrap(), batch.schema(), None).unwrap();
            writer.write(&batch).unwrap();
            writer.close().unwrap();
        };

        let dict: arrow::array::DictionaryArray<arrow::datatypes::Int32Type> =
            vec![Some("a"), None, Some("b")].into_iter().collect();
        for col in [
            Arc::new(StringArray::from(vec![Some("a"), None, Some("b")])) as ArrayRef,
            Arc::new(LargeStringArray::from(vec![Some("a"), None, Some("b")])),
            Arc::new(StringViewArray::from(vec![Some("a"), None, Some("b")])),
            Arc::new(dict),
        ] {
            let ty = col.data_type().clone();
            write(col);
            assert_eq!(load_openlr_codes(&path).unwrap(), vec!["a", "b"], "{ty:?}");
        }

        write(Arc::new(Int64Array::from(vec![1i64])));
        assert!(load_openlr_codes(&path).is_err());
        // No rows and no openlr column: still an error.
        let empty = RecordBatch::try_new(
            Arc::new(Schema::new(vec![Field::new(
                "other",
                DataType::Int64,
                true,
            )])),
            vec![Arc::new(Int64Array::from(Vec::<i64>::new()))],
        )
        .unwrap();
        let mut writer =
            ArrowWriter::try_new(File::create(&path).unwrap(), empty.schema(), None).unwrap();
        writer.write(&empty).unwrap();
        writer.close().unwrap();
        assert!(load_openlr_codes(&path).is_err());
        std::fs::remove_file(&path).ok();
    }
}
