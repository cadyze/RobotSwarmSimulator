using System.Globalization;
using System.IO;

namespace RobotSwarmSimulator;

/// <summary>Creates a portable SVG visit heatmap from the CSV movement log.</summary>
public sealed class SvgHeatmapProducer
{
    public string Create(string movementsCsvPath, int size, (int X, int Y) target, string outputPath)
    {
        var visits = new int[size, size];
        foreach (var line in File.ReadLines(movementsCsvPath).Skip(1))
        {
            var cells = line.Split(',');
            if (cells.Length < 6 || !int.TryParse(cells[2], out var x) || !int.TryParse(cells[3], out var y)) continue;
            if (x >= 0 && x < size && y >= 0 && y < size) visits[x, y]++;
        }
        var maximum = Math.Max(1, visits.Cast<int>().Max());
        const int cell = 22, margin = 50;
        using var writer = new StreamWriter(outputPath);
        writer.WriteLine($"<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{size * cell + 2 * margin}\" height=\"{size * cell + 2 * margin}\" viewBox=\"0 0 {size * cell + 2 * margin} {size * cell + 2 * margin}\">");
        writer.WriteLine("<rect width=\"100%\" height=\"100%\" fill=\"white\"/><text x=\"10\" y=\"22\" font-family=\"sans-serif\" font-size=\"16\">Robot position visits</text>");
        for (var y = 0; y < size; y++) for (var x = 0; x < size; x++)
        {
            var ratio = visits[x, y] / (double)maximum;
            var color = ViridisLike(ratio);
            var drawY = margin + (size - 1 - y) * cell;
            var outline = (x, y) == target ? "#d62728" : "#ffffff";
            var width = (x, y) == target ? 3 : 1;
            writer.WriteLine($"<rect x=\"{margin + x * cell}\" y=\"{drawY}\" width=\"{cell}\" height=\"{cell}\" fill=\"{color}\" stroke=\"{outline}\" stroke-width=\"{width}\"><title>x={x}, y={y}, visits={visits[x, y]}</title></rect>");
        }
        writer.WriteLine("</svg>");
        return outputPath;
    }

    private static string ViridisLike(double value)
    {
        var r = (int)Math.Round(68 + 185 * value); var g = (int)Math.Round(1 + 230 * value); var b = (int)Math.Round(84 - 47 * value);
        return $"rgb({r.ToString(CultureInfo.InvariantCulture)},{g.ToString(CultureInfo.InvariantCulture)},{b.ToString(CultureInfo.InvariantCulture)})";
    }
}
