$HASH = git rev-parse --short HEAD
$COMMIT_CLEAN = git describe --tags --always HEAD
$COMMIT_DIRTY = git describe --dirty --tags --always
$IS_DIRTY = 1
if( $COMMIT_CLEAN -eq $COMMIT_DIRTY )
{
	$IS_DIRTY=0
}
echo "const uint32_t GIT_INFO = 0x$HASH; const uint8_t IS_UNCOMMITTED = $IS_DIRTY;" | Out-File -Encoding ASCII -FilePath ../gitcommit.h