// =====================================================================
// SynROV Processing - Image stream utilities
// ---------------------------------------------------------------------
// Purpose:
//   Shared JPEG/Base64 encoding for browser and AI perception streams.
//   Encoding operates on a copy so one consumer can resize a frame without
//   mutating the source image used by another stream.
// =====================================================================

String encodePImageJpegBase64(PImage src, int targetW, int targetH, int maxBase64Chars) {
  if (src == null) return "";
  try {
    PImage frame = src.get();
    int safeW = max(64, targetW);
    int safeH = max(64, targetH);
    if (frame.width != safeW || frame.height != safeH) frame.resize(safeW, safeH);

    BufferedImage bimg = pimageToBuffered(frame);
    ByteArrayOutputStream baos = new ByteArrayOutputStream(96 * 1024);
    ImageIO.write(bimg, "jpg", baos);
    byte[] jpgBytes = baos.toByteArray();
    baos.close();

    String base64 = encodeBase64(jpgBytes);
    if (maxBase64Chars > 0 && base64.length() > maxBase64Chars) {
      println("[SynROV][Stream] JPEG frame too large: " + base64.length() + " Base64 chars");
      return "";
    }
    return base64;
  }
  catch (Exception e) {
    println("[SynROV][Stream] JPEG encode failed: " + e.getMessage());
    return "";
  }
}

// Converts a Processing image to an RGB BufferedImage for JPEG encoding.
static BufferedImage pimageToBuffered(PImage src) {
  src.loadPixels();
  int w = src.width, h = src.height;
  int[] rgb = new int[w * h];
  for (int i = 0; i < rgb.length; i++) {
    rgb[i] = src.pixels[i] & 0x00FFFFFF;
  }
  BufferedImage out = new BufferedImage(w, h, BufferedImage.TYPE_INT_RGB);
  out.setRGB(0, 0, w, h, rgb, 0, w);
  return out;
}


// Decodes a JPEG Base64 payload received from a physical robot camera.
PImage decodeJpegBase64ToPImage(String encoded) {
  if (encoded == null || encoded.length() == 0) return null;
  try {
    byte[] bytes = decodeBase64(encoded);
    if (bytes == null || bytes.length == 0) return null;
    BufferedImage bimg = ImageIO.read(new ByteArrayInputStream(bytes));
    if (bimg == null) return null;

    PImage out = new PImage(bimg.getWidth(), bimg.getHeight(), ARGB);
    out.loadPixels();
    bimg.getRGB(0, 0, bimg.getWidth(), bimg.getHeight(), out.pixels, 0, bimg.getWidth());
    out.updatePixels();
    return out;
  }
  catch (Exception e) {
    println("[SynROV][RobotCamera] JPEG decode failed: " + e.getMessage());
    return null;
  }
}
