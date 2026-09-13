using System.IO;
using System.Diagnostics;

namespace RobotSwarmSimulator;

/// <summary>Creates a 1280 x 720 H.264 animation from the triangular-lattice renderer.</summary>
public sealed class GifAnimationProducer
{
    private const int Width = 1280;
    private const int Height = 720;

    public string Create(string movementsCsvPath, int size, (int X, int Y) target, int trial, string outputPath)
    {
        var path = ReadTrial(movementsCsvPath, trial);
        if (path.Count == 0) throw new InvalidOperationException($"No movements were recorded for trial {trial}.");
        var scale = Math.Min((Width - 160.0) / (1.5 * Math.Max(1, size - 1)), (Height - 160.0) / Math.Max(1, size - 1));
        var baseGrid = DrawTriangularGrid(size, scale, target);
        using var encoder = StartMp4Encoder(outputPath);
        for (var frame = 0; frame < path.Count; frame++) WriteRgbFrame(encoder.StandardInput.BaseStream, DrawFrame(size, scale, baseGrid, target, path, frame));
        var finalFrame = DrawFrame(size, scale, baseGrid, target, path, path.Count - 1);
        for (var index = 0; index < 10; index++) WriteRgbFrame(encoder.StandardInput.BaseStream, finalFrame);
        encoder.StandardInput.Close();
        encoder.WaitForExit();
        if (encoder.ExitCode != 0) throw new InvalidOperationException($"FFmpeg could not encode '{outputPath}' (exit code {encoder.ExitCode}).");
        return outputPath;
    }

    private static Process StartMp4Encoder(string outputPath)
    {
        var relative = Path.Combine("tools", "ffmpeg", "bin", "ffmpeg.exe");
        var roots = ParentDirectories(Directory.GetCurrentDirectory()).Concat(ParentDirectories(AppContext.BaseDirectory));
        var ffmpeg = roots.Select(root => Path.Combine(root.FullName, relative)).Append(Path.Combine(AppContext.BaseDirectory, "ffmpeg.exe")).FirstOrDefault(File.Exists)
            ?? throw new FileNotFoundException("FFmpeg was not found. Place ffmpeg.exe under tools\\ffmpeg\\bin at the project root.");
        var info = new ProcessStartInfo(ffmpeg, $"-loglevel error -y -f rawvideo -pix_fmt rgb24 -s {Width}x{Height} -r 10 -i - -c:v libx264 -preset medium -crf 20 -pix_fmt yuv420p -movflags +faststart \"{outputPath}\"") { UseShellExecute = false, RedirectStandardInput = true, CreateNoWindow = true };
        return Process.Start(info) ?? throw new InvalidOperationException("Could not start FFmpeg.");
    }

    private static IEnumerable<DirectoryInfo> ParentDirectories(string path)
    {
        for (var directory = new DirectoryInfo(path); directory is not null; directory = directory.Parent) yield return directory;
    }

    private static void WriteRgbFrame(Stream output, byte[] pixels)
    {
        var colors = new (byte R, byte G, byte B)[] { (255, 255, 255), (180, 180, 180), (214, 39, 40), (198, 219, 239), (107, 174, 214), (33, 113, 181), (23, 190, 207) };
        var rgb = new byte[pixels.Length * 3];
        for (var index = 0; index < pixels.Length; index++) { var color = colors[pixels[index]]; var offset = index * 3; rgb[offset] = color.R; rgb[offset + 1] = color.G; rgb[offset + 2] = color.B; }
        output.Write(rgb);
    }

    private static List<(int X, int Y, int Heading)> ReadTrial(string csv, int trial)
    {
        var result = new List<(int X, int Y, int Heading)>();
        foreach (var line in File.ReadLines(csv).Skip(1))
        {
            var fields = line.Split(',');
            if (fields.Length < 6 || !int.TryParse(fields[0], out var rowTrial) || rowTrial != trial) continue;
            if (int.TryParse(fields[2], out var x) && int.TryParse(fields[3], out var y) && int.TryParse(fields[4], out var heading)) result.Add((X: x, Y: y, Heading: heading));
        }
        return result;
    }

    private static byte[] DrawFrame(int size, double scale, byte[] baseGrid, (int X, int Y) target, List<(int X, int Y, int Heading)> path, int frame)
    {
        var pixels = (byte[])baseGrid.Clone();
        var firstTrailStep = Math.Max(0, frame - 3);
        for (var index = firstTrailStep; index < frame; index++)
        {
            var color = (byte)(3 + index - firstTrailStep);
            DrawLine(pixels, ToPixel(path[index].X, path[index].Y, size, scale), ToPixel(path[index + 1].X, path[index + 1].Y, size, scale), color);
            PaintCircle(pixels, size, scale, path[index].X, path[index].Y, Math.Max(4, scale * .10), color);
        }
        PaintHexagon(pixels, size, scale, target.X, target.Y, scale * .32, 2);
        var robot = path[frame];
        PaintCircle(pixels, size, scale, robot.X, robot.Y, Math.Max(6, scale * .14), 6);
        return pixels;
    }

    private static byte[] DrawTriangularGrid(int size, double scale, (int X, int Y) target)
    {
        var pixels = new byte[Width * Height];
        for (var y = 0; y < size; y++) for (var x = 0; x < size; x++)
            foreach (var (dx, dy) in new[] { (1, 0), (0, 1), (-1, 1) })
                if (x + dx >= 0 && x + dx < size && y + dy >= 0 && y + dy < size) DrawLine(pixels, ToPixel(x, y, size, scale), ToPixel(x + dx, y + dy, size, scale), 1);
        PaintHexagon(pixels, size, scale, target.X, target.Y, scale * .32, 2);
        return pixels;
    }

    private static (int X, int Y) ToPixel(int x, int y, int size, double scale)
    {
        var latticeWidth = 1.5 * (size - 1) * scale;
        var latticeHeight = (size - 1) * scale;
        return ((int)Math.Round((Width - latticeWidth) / 2 + (x + .5 * y) * scale), (int)Math.Round((Height - latticeHeight) / 2 + (size - 1 - y) * scale));
    }

    private static void DrawLine(byte[] pixels, (int X, int Y) start, (int X, int Y) end, byte color)
    {
        var dx = Math.Abs(end.X - start.X); var sx = start.X < end.X ? 1 : -1; var dy = -Math.Abs(end.Y - start.Y); var sy = start.Y < end.Y ? 1 : -1; var error = dx + dy;
        while (true) { pixels[start.Y * Width + start.X] = color; if (start == end) return; var twice = 2 * error; if (twice >= dy) { error += dy; start.X += sx; } if (twice <= dx) { error += dx; start.Y += sy; } }
    }

    private static void PaintCircle(byte[] pixels, int size, double scale, int x, int y, double radius, byte color)
    {
        var center = ToPixel(x, y, size, scale); var r = (int)Math.Ceiling(radius);
        for (var py = center.Y - r; py <= center.Y + r; py++) for (var px = center.X - r; px <= center.X + r; px++) if ((px - center.X) * (px - center.X) + (py - center.Y) * (py - center.Y) <= radius * radius) pixels[py * Width + px] = color;
    }

    private static void PaintHexagon(byte[] pixels, int size, double scale, int x, int y, double radius, byte color)
    {
        var center = ToPixel(x, y, size, scale); var r = (int)Math.Ceiling(radius);
        for (var py = center.Y - r; py <= center.Y + r; py++) for (var px = center.X - r; px <= center.X + r; px++) if (Math.Abs(px - center.X) + Math.Abs(py - center.Y) * .58 <= radius) pixels[py * Width + px] = color;
    }

    private static void PaintRobotTriangle(byte[] pixels, int size, double scale, int x, int y, int heading, byte color)
    {
        var center = ToPixel(x, y, size, scale); var directions = new[] { (1.0, 0.0), (.5, -1.0), (-.5, -1.0), (-1.0, 0.0), (-.5, 1.0), (.5, 1.0) }; var (dx, dy) = directions[heading];
        var length = Math.Sqrt(dx * dx + dy * dy); dx /= length; dy /= length; var perpendicular = (-dy, dx);
        var tip = (center.X + dx * scale * .36, center.Y + dy * scale * .36); var left = (center.X - dx * scale * .20 + perpendicular.Item1 * scale * .20, center.Y - dy * scale * .20 + perpendicular.Item2 * scale * .20); var right = (center.X - dx * scale * .20 - perpendicular.Item1 * scale * .20, center.Y - dy * scale * .20 - perpendicular.Item2 * scale * .20);
        var minX = (int)Math.Floor(Math.Min(tip.Item1, Math.Min(left.Item1, right.Item1))); var maxX = (int)Math.Ceiling(Math.Max(tip.Item1, Math.Max(left.Item1, right.Item1))); var minY = (int)Math.Floor(Math.Min(tip.Item2, Math.Min(left.Item2, right.Item2))); var maxY = (int)Math.Ceiling(Math.Max(tip.Item2, Math.Max(left.Item2, right.Item2)));
        for (var py = minY; py <= maxY; py++) for (var px = minX; px <= maxX; px++) if (InsideTriangle(px, py, tip, left, right)) pixels[py * Width + px] = color;
    }

    private static bool InsideTriangle(double px, double py, (double X, double Y) a, (double X, double Y) b, (double X, double Y) c)
    {
        static double Sign(double px, double py, (double X, double Y) u, (double X, double Y) v) => (px - v.X) * (u.Y - v.Y) - (u.X - v.X) * (py - v.Y);
        var first = Sign(px, py, a, b); var second = Sign(px, py, b, c); var third = Sign(px, py, c, a); return (first >= 0 && second >= 0 && third >= 0) || (first <= 0 && second <= 0 && third <= 0);
    }

    private sealed class GifWriter(Stream stream)
    {
        private readonly BinaryWriter _writer = new(stream);
        public void WriteHeader()
        {
            _writer.Write("GIF89a"u8); _writer.Write((ushort)Width); _writer.Write((ushort)Height); _writer.Write((byte)0xF7); _writer.Write((byte)0); _writer.Write((byte)0);
            var colors = new (byte R, byte G, byte B)[] { (255, 255, 255), (180, 180, 180), (214, 39, 40), (198, 219, 239), (107, 174, 214), (33, 113, 181), (23, 190, 207) };
            for (var i = 0; i < 256; i++) { var color = i < colors.Length ? colors[i] : (R: (byte)0, G: (byte)0, B: (byte)0); _writer.Write(color.R); _writer.Write(color.G); _writer.Write(color.B); }
            _writer.Write(new byte[] { 0x21, 0xFF, 0x0B }); _writer.Write("NETSCAPE2.0"u8); _writer.Write(new byte[] { 0x03, 0x01, 0x00, 0x00, 0x00 });
        }
        public void WriteFrame(byte[] pixels, int delay)
        {
            _writer.Write(new byte[] { 0x21, 0xF9, 0x04, 0x00, (byte)delay, (byte)(delay >> 8), 0, 0, 0x2C });
            _writer.Write((ushort)0); _writer.Write((ushort)0); _writer.Write((ushort)Width); _writer.Write((ushort)Height); _writer.Write((byte)0); _writer.Write((byte)8);
            var bits = new BitWriter(); foreach (var pixel in pixels) { bits.Write(256, 9); bits.Write(pixel, 9); } bits.Write(257, 9); WriteBlocks(bits.ToArray());
        }
        public void Finish() { _writer.Write((byte)0x3B); _writer.Flush(); }
        private void WriteBlocks(byte[] bytes) { for (var offset = 0; offset < bytes.Length; offset += 255) { var count = Math.Min(255, bytes.Length - offset); _writer.Write((byte)count); _writer.Write(bytes, offset, count); } _writer.Write((byte)0); }
        private sealed class BitWriter
        {
            private readonly List<byte> _bytes = []; private int _value, _bits;
            public void Write(int value, int width) { _value |= value << _bits; _bits += width; while (_bits >= 8) { _bytes.Add((byte)_value); _value >>= 8; _bits -= 8; } }
            public byte[] ToArray() { if (_bits > 0) _bytes.Add((byte)_value); return _bytes.ToArray(); }
        }
    }
}
