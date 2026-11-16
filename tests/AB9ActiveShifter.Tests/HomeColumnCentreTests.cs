using System;
using AB9ActiveShifter.Core;
using Xunit;

namespace AB9ActiveShifter.Tests
{
    /// <summary>
    /// Sliding the pattern so the home column stands on the axis centre. The base's own spring
    /// centres on the axis, and a four-column gate centred there has a barrier crest under the
    /// resting lever instead of a column - traced as a 16 Hz lateral bounce in neutral. These
    /// pin that the default moves nothing, that the shift is rigid and never pushes a column off
    /// the end of travel, and that with it on the lever's rest is a free place.
    /// </summary>
    public class HomeColumnCentreTests
    {
        private const int Centre = GateGeometry.AxisCenter;
        private const int Max = GateGeometry.AxisMax;

        private static readonly GatePattern[] HPatterns =
        {
            GatePattern.H7R, GatePattern.H6R, GatePattern.H5R, GatePattern.H6
        };

        private static EngineConfig Config(GatePattern pattern, int widthPct, bool centre, bool mirror = false)
        {
            return new EngineConfig
            {
                OverallGainPct = 100,
                PolarityConfirmed = true,
                Pattern = pattern,
                PatternWidthPct = widthPct,
                CentreHomeColumn = centre,
                MirrorColumns = mirror
            };
        }

        [Fact]
        public void OffIsTheDefaultAndMovesNothing()
        {
            Assert.False(new EngineConfig().CentreHomeColumn);
            Assert.False(new ShifterSettings().CentreHomeColumn);

            foreach (GatePattern pattern in HPatterns)
            {
                foreach (int width in new[] { 100, 75, 60, 40 })
                {
                    GateGeometry plain = new EngineConfig { Pattern = pattern, PatternWidthPct = width }.BuildGeometry();
                    GateGeometry off = Config(pattern, width, centre: false).BuildGeometry();

                    Assert.Equal(0, off.PatternShift);
                    Assert.False(off.HomeColumnCentreRepaired);
                    for (int c = 0; c < plain.ColumnCount; c++)
                        Assert.Equal(plain.ColumnTarget((Column)c), off.ColumnTarget((Column)c));
                }
            }
        }

        [Fact]
        public void AFourColumnGateRestsOnItsHomeColumn()
        {
            foreach (GatePattern pattern in new[] { GatePattern.H7R, GatePattern.H6R })
            {
                GateGeometry geo = Config(pattern, 60, centre: true).BuildGeometry();

                Assert.Equal(Centre, geo.ColumnTarget(geo.HomeColumn));
                Assert.False(geo.HomeColumnCentreRepaired);
                Assert.Equal(3, geo.GearFor(geo.HomeColumn, ShiftDir.Fwd));
                Assert.Equal(4, geo.GearFor(geo.HomeColumn, ShiftDir.Back));
                Assert.Equal(Column.C2, geo.ColumnAt(Centre));
            }
        }

        [Fact]
        public void TheShiftIsRigid()
        {
            // Every column moves by the same amount, so spacing - and with it every corridor,
            // barrier and face derived from it - is exactly what it was.
            GateGeometry plain = Config(GatePattern.H6R, 60, centre: false).BuildGeometry();
            GateGeometry moved = Config(GatePattern.H6R, 60, centre: true).BuildGeometry();

            Assert.NotEqual(0, moved.PatternShift);
            Assert.Equal(plain.ColumnSpacing, moved.ColumnSpacing);
            for (int c = 0; c < plain.ColumnCount; c++)
                Assert.Equal(plain.ColumnTarget((Column)c) + moved.PatternShift, moved.ColumnTarget((Column)c));
            for (int gap = 0; gap < plain.ColumnCount - 1; gap++)
                Assert.Equal(plain.BarrierCentre(gap) + moved.PatternShift, moved.BarrierCentre(gap));
            Assert.Equal(plain.LockoutCentre + moved.PatternShift, moved.LockoutCentre);
        }

        [Fact]
        public void MirroringMovesItTheOtherWay()
        {
            GateGeometry normal = Config(GatePattern.H6R, 60, centre: true).BuildGeometry();
            GateGeometry mirrored = Config(GatePattern.H6R, 60, centre: true, mirror: true).BuildGeometry();

            Assert.Equal(Centre, mirrored.ColumnTarget(mirrored.HomeColumn));
            Assert.Equal(Column.C3, mirrored.HomeColumn);
            Assert.Equal(Math.Sign(normal.PatternShift), -Math.Sign(mirrored.PatternShift));
        }

        [Fact]
        public void NoColumnIsEverPushedOffTheAxis()
        {
            foreach (GatePattern pattern in HPatterns)
            {
                foreach (bool mirror in new[] { false, true })
                {
                    for (int width = GateGeometry.MinPatternWidthPct; width <= 100; width++)
                    {
                        GateGeometry geo = Config(pattern, width, centre: true, mirror: mirror).BuildGeometry();
                        Assert.InRange(geo.ColumnTarget(Column.C1), 0, Max);
                        Assert.InRange(geo.ColumnTarget((Column)(geo.ColumnCount - 1)), 0, Max);
                    }
                }
            }
        }

        [Fact]
        public void APatternTooWideMovesAsFarAsItCanAndSaysSo()
        {
            GateGeometry full = Config(GatePattern.H6R, 100, centre: true).BuildGeometry();
            Assert.Equal(0, full.PatternShift);
            Assert.True(full.HomeColumnCentreRepaired);

            GateGeometry wide = Config(GatePattern.H6R, 90, centre: true).BuildGeometry();
            Assert.True(wide.PatternShift > 0);
            Assert.True(wide.HomeColumnCentreRepaired);
            Assert.Equal(Max, wide.ColumnTarget(Column.C4));

            GateGeometry fits = Config(GatePattern.H6R, 75, centre: true).BuildGeometry();
            Assert.False(fits.HomeColumnCentreRepaired);
        }

        [Fact]
        public void AThreeColumnGateWasAlreadyThere()
        {
            foreach (GatePattern pattern in new[] { GatePattern.H5R, GatePattern.H6 })
            {
                GateGeometry geo = Config(pattern, 60, centre: true).BuildGeometry();
                Assert.InRange(Math.Abs(geo.PatternShift), 0, 1);
                Assert.False(geo.HomeColumnCentreRepaired);
            }
        }

        [Fact]
        public void TheOtherPatternsIgnoreIt()
        {
            foreach (GatePattern pattern in new[] { GatePattern.Sequential, GatePattern.Prnd, GatePattern.Handbrake })
            {
                GateGeometry geo = Config(pattern, 60, centre: true).BuildGeometry();
                Assert.Equal(0, geo.PatternShift);
                Assert.False(geo.HomeColumnCentreRepaired);
            }
        }

        [Fact]
        public void TheRestingLeverIsNoLongerOnABarrier()
        {
            // The traced fault as a property. Centred on the axis, a 6+R gate pushes a lever
            // resting a little off the centre away from it - the barrier - which is what the
            // base's spring then fights. With the home column there, the same places are free.
            EngineConfig plainCfg = Config(GatePattern.H6R, 60, centre: false);
            plainCfg.BarrierForcePct = 40; plainCfg.BarrierWidth = 400;   // the traced tune
            var plain = new ForceComposer(plainCfg.BuildGeometry(), plainCfg);

            EngineConfig movedCfg = Config(GatePattern.H6R, 60, centre: true);
            movedCfg.BarrierForcePct = 40; movedCfg.BarrierWidth = 400;
            var moved = new ForceComposer(movedCfg.BuildGeometry(), movedCfg);

            int worstPlain = 0, worstMoved = 0;
            for (int dx = -600; dx <= 600; dx += 50)
            {
                worstPlain = Math.Max(worstPlain, Math.Abs(plain.BarrierForceIn(Centre + dx, Centre)));
                worstMoved = Math.Max(worstMoved, Math.Abs(moved.BarrierForceIn(Centre + dx, Centre)));
            }

            Assert.True(worstPlain > 2000, "the centred gate should have its barrier at the centre: " + worstPlain);
            Assert.True(worstMoved < worstPlain / 50, "the rest should be clear of every barrier: " + worstMoved + " vs " + worstPlain);
        }

        [Fact]
        public void ItReachesTheEngineAndASharedProfile()
        {
            var settings = new ShifterSettings { Pattern = GatePattern.H6R, PatternWidthPct = 60, CentreHomeColumn = true };
            Assert.True(settings.ToEngineConfig().CentreHomeColumn);
            Assert.True(ProfileTransfer.IsTuning("CentreHomeColumn"));
        }
    }
}
