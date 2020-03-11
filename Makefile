DDC_DEFAULTS=-DFEATURE_BOARD_DEFAULT=feature_board_usbi2s \
	-DFEATURE_IMAGE_DEFAULT=feature_image_uac2_audio \
	-DFEATURE_IN_DEFAULT=feature_in_normal \
	-DFEATURE_OUT_DEFAULT=feature_out_normal \
	-DFEATURE_ADC_DEFAULT=feature_adc_none \
	-DFEATURE_DAC_DEFAULT=feature_dac_generic \
	-DFEATURE_LCD_DEFAULT=feature_lcd_none \
	-DFEATURE_LOG_DEFAULT=feature_log_none \
	-DFEATURE_FILTER_DEFAULT=feature_filter_fir \
	-DFEATURE_QUIRK_DEFAULT=feature_quirk_none \
	-DFEATURE_VOLUME_CTRL \
	-DFEATURE_PRODUCT_AB1x \
	-DHW_GEN_AB1X

all:: main

main::
	CFLAGS="$(DDC_DEFAULTS)" ./make-widget

clean::
	cd Release && make clean

write:: main
	./program_widget
