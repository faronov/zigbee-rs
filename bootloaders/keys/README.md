# Default loader keys

Public halves of the project's own ECDSA P-256 image-signing key (the same
key as `products/bl702-xt-zb1/keys/`). Product layout files
(`products/*/boot/*.layout`) name `own-ec-p256.xy.hex` as their default
`key-file`; a build overrides it with `<PREFIX>_OTA_KEY_XY` (and adds keys
with `<PREFIX>_OTA_KEY2_XY`/`_KEY3_XY`).

- `own-ec-p256.xy.hex`: raw `X || Y`, 128 hex digits, what the loaders embed.
- `own-ec-p256.pub.pem`: the same public key for `imgtool verify`.

The private key is never committed.
