use super::*;

/// Temperature report 0 (timeout 60 s): Temperature Measurement server,
/// MeasuredValue (int16) at offset 0.
pub(crate) const TEMPERATURE: [u8; 13] = [
    0x00, 0x01, 0x3C, 0x00, 0x08, // id 0, timeout 60 s, 8 octets
    0x08, 0x02, 0x04, // server, 1 record, cluster 0x0402
    0x00, 0x00, 0x29, 0x11, 0x00, // MeasuredValue int16, reported at 0
];

/// Report 1: Relative Humidity MeasuredValue at offset 0 plus a fixed
/// ClusterRevision, and a manufacturer-specific attribute at offset 2.
pub(crate) const HUMIDITY: [u8; 27] = [
    0x01, 0x00, 0x18, // id 1, no timeout, 24 octets
    0x09, 0x05, 0x04, // server, 2 records, cluster 0x0405
    0x00, 0x00, 0x21, 0x11, 0x00, // MeasuredValue uint16 at 0
    0xFD, 0xFF, 0x21, 0x22, 0x01, 0x00, // ClusterRevision = 1 (fixed)
    0x18, 0x00, 0xFC, 0x34, 0x12, // server, manufacturer 0x1234, 0xFC00
    0x01, 0x00, 0x20, 0x11, 0x02, // attribute 0x0001 uint8 at 2
];

pub(crate) fn frame(total: u8, reports: &[&[u8]]) -> heapless::Vec<u8, 104> {
    let mut v = heapless::Vec::new();
    v.push(total).unwrap();
    v.push(reports.len() as u8).unwrap();
    for r in reports {
        v.extend_from_slice(r).unwrap();
    }
    v
}

#[test]
fn decodes_two_reports() {
    let payload = frame(2, &[&TEMPERATURE, &HUMIDITY]);
    let d = ApplicationDescription::decode(&payload).unwrap();
    assert_eq!((d.total_reports, d.report_count), (2, 2));
    let reports: heapless::Vec<_, 4> = d.iter().collect();
    assert_eq!(reports.len(), 2);
    assert_eq!(reports[0].report_id, 0);
    assert_eq!(reports[0].timeout, Some(60));
    assert_eq!(reports[0].raw, &TEMPERATURE);
    let dp: heapless::Vec<_, 4> = reports[0].iter().collect();
    assert_eq!(dp.len(), 1);
    assert!(dp[0].server);
    assert_eq!((dp[0].cluster_id, dp[0].manufacturer_id), (0x0402, None));
    let rec: heapless::Vec<_, 4> = dp[0].iter().collect();
    assert_eq!(
        rec.as_slice(),
        &[AttributeRecord {
            attribute_id: 0x0000,
            data_type: 0x29,
            offset: Some(0),
            value: None,
        }]
    );

    assert_eq!(reports[1].report_id, 1);
    assert_eq!(reports[1].timeout, None);
    assert_eq!(reports[1].raw, &HUMIDITY);
    let dp: heapless::Vec<_, 4> = reports[1].iter().collect();
    assert_eq!(dp.len(), 2);
    assert_eq!(dp[0].cluster_id, 0x0405);
    let rec: heapless::Vec<_, 4> = dp[0].iter().collect();
    assert_eq!(rec.len(), 2);
    assert_eq!(rec[1].attribute_id, 0xFFFD);
    assert_eq!(
        (rec[1].offset, rec[1].value),
        (None, Some(&[0x01, 0x00][..]))
    );
    assert_eq!(
        (dp[1].cluster_id, dp[1].manufacturer_id),
        (0xFC00, Some(0x1234))
    );
    let rec: heapless::Vec<_, 4> = dp[1].iter().collect();
    assert_eq!((rec[0].attribute_id, rec[0].offset), (0x0001, Some(2)));
}

#[test]
fn one_report_of_several() {
    let payload = frame(3, &[&HUMIDITY]);
    let d = ApplicationDescription::decode(&payload).unwrap();
    assert_eq!((d.total_reports, d.report_count), (3, 1));
    assert_eq!(d.iter().next().unwrap().report_id, 1);
}

#[test]
fn rejects_malformed_descriptions() {
    // No report, or more reports in the frame than in total.
    assert_eq!(
        ApplicationDescription::decode(&[1, 0]),
        Err(Error::Unsupported)
    );
    let payload = frame(1, &[&TEMPERATURE, &HUMIDITY]);
    assert_eq!(
        ApplicationDescription::decode(&payload),
        Err(Error::Unsupported)
    );
    // Truncated descriptor and trailing octets.
    let payload = frame(1, &[&TEMPERATURE[..12]]);
    assert_eq!(
        ApplicationDescription::decode(&payload),
        Err(Error::Truncated)
    );
    let mut payload = frame(1, &[&TEMPERATURE]);
    payload.push(0).unwrap();
    assert_eq!(
        ApplicationDescription::decode(&payload),
        Err(Error::TrailingBytes)
    );
    // A report without data points is shorter than MIN_REPORT_LEN.
    assert_eq!(
        ApplicationDescription::decode(&[1, 1, 0, 0, 0]),
        Err(Error::Unsupported)
    );
    // "Value present" without a value, and a value without the flag.
    let mut bad = TEMPERATURE;
    bad[11] = 0x31;
    let payload = frame(1, &[&bad]);
    assert_eq!(
        ApplicationDescription::decode(&payload),
        Err(Error::Unsupported)
    );
    let mut bad = TEMPERATURE;
    bad[11] = 0x01;
    let payload = frame(1, &[&bad]);
    assert_eq!(
        ApplicationDescription::decode(&payload),
        Err(Error::Unsupported)
    );
    // A data point that overruns the report's remaining length.
    let mut bad = TEMPERATURE;
    bad[4] = 7;
    let payload = frame(1, &[&bad[..12]]);
    assert_eq!(
        ApplicationDescription::decode(&payload),
        Err(Error::Truncated)
    );
}

#[test]
fn pairing_configuration_parts() {
    let mut reports = heapless::Vec::<u8, 64>::new();
    reports.extend_from_slice(&TEMPERATURE).unwrap();
    reports.extend_from_slice(&HUMIDITY).unwrap();
    let d = ApplicationDescription::from_parts(2, 2, &reports).unwrap();
    assert_eq!(d.iter().count(), 2);
    assert_eq!(
        ApplicationDescription::from_parts(2, 1, &reports),
        Err(Error::TrailingBytes)
    );
}
